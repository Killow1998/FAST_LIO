#!/usr/bin/env python3
"""Native ROS replay qualification; input contains only FAST-LIO-consumed sensor fields."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import xmlrpc.client

import numpy as np
import yaml


def records(path):
    if path:
        with path.open('rb') as stream:
            assert stream.read(8) == b'FLIO1\0\0\0'
            while True:
                kind = stream.read(1)
                if not kind:
                    break
                if kind == b'I':
                    data = struct.unpack('<Q6d', stream.read(56))
                    yield kind, data[0], data[1:]
                else:
                    assert kind == b'L'
                    stamp, count = struct.unpack('<QI', stream.read(12))
                    data = stream.read(count * 19)
                    assert len(data) == count * 19
                    yield kind, stamp, data
    else:
        for tick in range(1402):
            stamp = tick * 5000000
            seconds = stamp / 1e9
            acceleration = .1 if 1.5 <= seconds < 2.5 else 0.
            yield b'I', stamp, (acceleration, 0., 9.81, 0., 0., 0.)
            if tick % 20:
                continue
            elapsed = max(0., seconds - 1.5)
            position = .05 * elapsed**2 if elapsed <= 1. else .05 + .1 * (elapsed - 1.)
            points = []
            for i in range(9):
                for j in range(9):
                    for x, y, z in ((1.+i*.2-position, -1.+j*.2, -1.),
                                    (3.-position, -1.+i*.2, -1.+j*.2),
                                    (1.+i*.2-position, 2., -1.+j*.2)):
                        points.append(struct.pack('<IfffBBB', 0, x, y, z, 10, 0x10, 0))
            yield b'L', stamp, b''.join(points)


class Transport:
    def __init__(self, ros, lidar_type):
        self.ros = ros
        self.subscribers = []
        if ros == 'humble':
            import rclpy
            from sensor_msgs.msg import Imu, PointCloud2, PointField
            from livox_ros_driver2.msg import CustomMsg, CustomPoint
            from nav_msgs.msg import Odometry, Path as PathMsg
            from geometry_msgs.msg import Vector3Stamped
            from std_msgs.msg import UInt64
            from tf2_msgs.msg import TFMessage
            rclpy.init()
            self.api = rclpy
            self.node = rclpy.create_node('fast_lio_pipeline_probe')
        else:
            import rospy
            from sensor_msgs.msg import Imu, PointCloud2, PointField
            from livox_ros_driver.msg import CustomMsg, CustomPoint
            from nav_msgs.msg import Odometry, Path as PathMsg
            from geometry_msgs.msg import Vector3Stamped
            from std_msgs.msg import UInt64
            from tf2_msgs.msg import TFMessage
            rospy.init_node('fast_lio_pipeline_probe', disable_signals=True)
            self.api = rospy
        self.Imu, self.Lidar, self.Point = Imu, CustomMsg, CustomPoint
        self.TF = TFMessage
        self.PointField = PointField
        self.Odom, self.Cloud, self.Path, self.Vector, self.Ack = Odometry, PointCloud2, PathMsg, Vector3Stamped, UInt64
        self.imu = self.publisher('/livox/imu', Imu)
        self.lidar = self.publisher('/livox/lidar', CustomMsg if lidar_type == 1 else PointCloud2)

    def publisher(self, topic, kind):
        if self.ros == 'humble':
            return self.node.create_publisher(kind, topic, 200)
        return self.api.Publisher(topic, kind, queue_size=200)

    def subscribe(self, topic, kind, callback):
        sub = self.node.create_subscription(kind, topic, callback, 200) if self.ros == 'humble' else self.api.Subscriber(topic, kind, callback, queue_size=200)
        self.subscribers.append(sub)

    def spin(self, seconds=.001):
        seconds = max(0., seconds)
        if self.ros == 'humble':
            self.api.spin_once(self.node, timeout_sec=seconds)
        else:
            time.sleep(seconds)

    def ready(self):
        method = 'get_subscription_count' if self.ros == 'humble' else 'get_num_connections'
        return all(getattr(pub, method)() > 0 for pub in (self.imu, self.lidar))

    def stamp(self, message, stamp):
        if self.ros == 'humble':
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1000000000)
        else:
            message.header.stamp = self.api.Time(*divmod(stamp, 1000000000))

    def nanoseconds(self, message):
        stamp = message.header.stamp
        return stamp.sec * 1000000000 + stamp.nanosec if self.ros == 'humble' else stamp.to_nsec()

    def close(self):
        if self.ros == 'humble':
            self.node.destroy_node()
            self.api.shutdown()
        else:
            self.api.signal_shutdown('qualification complete')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ros', choices=['humble', 'noetic'], required=True)
    parser.add_argument('--node', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--input', type=Path)
    parser.add_argument('--config', type=Path)
    parser.add_argument('--rate', type=float, default=2.)
    parser.add_argument('--lidar-type', type=int, choices=[1,2,3,4], default=1)
    args = parser.parse_args()
    if args.input and args.lidar_type != 1:
        parser.error('Recorded inputs retain their original Livox format')
    args.output.mkdir(parents=True, exist_ok=False)
    config = yaml.safe_load(args.config.read_text())['laserMapping']['ros__parameters'] if args.config else {
        'common': {'gravity_m_s2': 9.81}, 'mapping': {'extrinsic_T': [0.,0.,0.], 'extrinsic_R': [1.,0.,0.,0.,1.,0.,0.,0.,1.], 'extrinsic_est_en': False},
        'preprocess': {'lidar_type': 1, 'scan_line': 1, 'blind': .01}}
    config.update(filter_size_surf=.1, filter_size_map=.1, point_filter_num=2 if args.input else 1,
                  max_iteration=4, runtime_pos_log_enable=False)
    config.setdefault("cube_side_length", 1000.)
    config.setdefault("preprocess", {})["lidar_type"] = args.lidar_type
    if args.lidar_type in (2,3):
        config["preprocess"]["timestamp_unit"] = 0 if args.lidar_type == 2 else 3
    config['pcd_save'] = {'pcd_save_en': False, 'interval': -1}
    config['publish'] = {'path_en': True, 'scan_publish_en': True, 'dense_publish_en': True, 'scan_bodyframe_pub_en': True}
    config.setdefault('common', {}).update(lid_topic='/livox/lidar', imu_topic='/livox/imu', world_frame='camera_init', body_frame='body', replay_lidar_ack_topic='/probe/ack')
    if args.input:
        config['common']['gravity_m_s2'] = 9.80665
    parameters = args.output / 'parameters.yaml'
    parameters.write_text(yaml.safe_dump({'/**': {'ros__parameters': config}} if args.ros == 'humble' else config))
    processes = []
    errors = []
    stream = (args.output / 'processes.log').open('w')
    transport = None
    files = []
    result = None
    try:
        if args.ros == 'noetic':
            os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11391'
            os.environ['ROS_IP'] = '127.0.0.1'
            os.environ.pop('ROS_HOSTNAME', None)
            os.environ['ROS_HOME'] = str(args.output / 'ros')
            master = subprocess.Popen(['roscore', '-p', '11391'], stdout=stream, stderr=stream, start_new_session=True)
            processes.append(master)
            deadline = time.monotonic()+15
            while time.monotonic() < deadline:
                try:
                    if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/probe')[0] == 1:
                        break
                except OSError:
                    time.sleep(.05)
            import rosparam
            rosparam.upload_params('/', config)
        transport = Transport(args.ros, args.lidar_type)
        counts = {'ack': 0, 'odom': 0, 'imu_odom': 0, 'body': 0, 'world': 0, 'path': 0, 'gravity': 0, 'tf': 0}
        pose_file = (args.output / 'odometry.csv').open('w');files.append(pose_file)
        pose_csv = csv.writer(pose_file)
        imu_file = (args.output / 'imu_odometry.csv').open('w');files.append(imu_file)
        imu_csv = csv.writer(imu_file)
        cloud_file = (args.output / 'body_clouds.bin').open('wb');files.append(cloud_file)
        gravity_file = (args.output / 'gravity.csv').open('w');files.append(gravity_file)
        gravity_csv = csv.writer(gravity_file)
        tf_file = (args.output / 'tf.csv').open('w');files.append(tf_file)
        tf_csv = csv.writer(tf_file)
        body_fields = ['x','y','z','intensity','normal_x','normal_y','normal_z','curvature']
        pending = {}
        ack_latencies = []
        def acknowledge(message):
            sequence = int(message.data)
            counts['ack'] = sequence
            if sequence in pending:
                ack_latencies.append(time.monotonic()-pending.pop(sequence))
        def gravity(message):
            counts['gravity'] += 1
            if message.header.frame_id != 'camera_init':
                errors.append('gravity frame mismatch')
            gravity_csv.writerow([transport.nanoseconds(message),message.vector.x,message.vector.y,message.vector.z])
        def transforms(message):
            for transform in message.transforms:
                counts['tf'] += 1
                if transform.header.frame_id != 'camera_init' or transform.child_frame_id != 'body':
                    errors.append('TF frame mismatch')
                p=transform.transform.translation;q=transform.transform.rotation
                tf_csv.writerow([transport.nanoseconds(transform),p.x,p.y,p.z,q.x,q.y,q.z,q.w])
        def pose(message, key, writer):
            p=message.pose.pose.position;q=message.pose.pose.orientation;v=message.twist.twist.linear;w=message.twist.twist.angular
            counts[key] += 1
            if message.header.frame_id != 'camera_init' or message.child_frame_id != 'body':
                errors.append('odometry frame mismatch')
            if not np.isfinite([p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z,*message.pose.covariance]).all():
                errors.append('nonfinite odometry')
            writer.writerow([transport.nanoseconds(message),p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z,*message.pose.covariance])
        def cloud(message, key):
            counts[key] += 1
            if message.header.frame_id != ('body' if key == 'body' else 'camera_init'):
                errors.append('cloud frame mismatch')
            if key != 'body':
                return
            offsets = {field.name: field.offset for field in message.fields}
            dtype = np.dtype({'names': body_fields, 'formats': ['<f4']*len(body_fields),
                              'offsets': [offsets[x] for x in body_fields], 'itemsize': message.point_step})
            points = np.frombuffer(message.data, dtype=dtype, count=message.width*message.height)
            dense = np.column_stack([points[name] for name in body_fields]).astype('<f4', copy=False)
            if not np.isfinite(dense).all():
                errors.append('nonfinite cloud')
            cloud_file.write(struct.pack('<QI', transport.nanoseconds(message), len(dense)))
            cloud_file.write(dense.tobytes())
        transport.subscribe('/probe/ack', transport.Ack, acknowledge)
        transport.subscribe('/Odometry', transport.Odom, lambda m: pose(m,'odom',pose_csv))
        transport.subscribe('/Odom_imu_state', transport.Odom, lambda m: pose(m,'imu_odom',imu_csv))
        transport.subscribe('/cloud_registered_body', transport.Cloud, lambda m: cloud(m,'body'))
        transport.subscribe('/cloud_registered', transport.Cloud, lambda m: cloud(m,'world'))
        transport.subscribe('/path', transport.Path, lambda m: counts.__setitem__('path',counts['path']+1))
        transport.subscribe('/lio_gravity', transport.Vector, gravity)
        transport.subscribe('/tf', transport.TF, transforms)
        command = [str(args.node.resolve())]
        if args.ros == 'humble':
            command += ['--ros-args','--params-file',str(parameters.resolve())]
        node = subprocess.Popen(command, stdout=stream, stderr=stream, start_new_session=True)
        processes.append(node)
        deadline = time.monotonic()+20
        while not transport.ready() and time.monotonic() < deadline:
            transport.spin(.02)
            assert node.poll() is None, 'node exited during discovery'
        assert transport.ready(), 'input subscriptions missing'
        for _ in range(20):
            transport.spin(.01)
        sent = {'imu':0,'lidar':0}
        initial_stat=Path('/proc/{}/stat'.format(node.pid)).read_text().split()
        initial_cpu=(int(initial_stat[13])+int(initial_stat[14]))/os.sysconf('SC_CLK_TCK')
        start = time.monotonic()
        first_stamp = None
        progress_stamp = 0
        for kind, stamp, data in records(args.input):
            if first_stamp is None:
                first_stamp = stamp
            progress_stamp = max(progress_stamp,stamp)
            target = start + (progress_stamp-first_stamp)/1e9/args.rate
            while time.monotonic() < target:
                transport.spin(min(.002,target-time.monotonic()))
            if kind == b'I':
                message=transport.Imu();transport.stamp(message,stamp);message.header.frame_id='body'
                message.linear_acceleration.x,message.linear_acceleration.y,message.linear_acceleration.z=data[:3]
                message.angular_velocity.x,message.angular_velocity.y,message.angular_velocity.z=data[3:]
                transport.imu.publish(message);sent['imu']+=1
            else:
                deadline=time.monotonic()+10
                while counts['ack'] < sent['lidar'] and time.monotonic() < deadline:
                    transport.spin(.002)
                assert counts['ack'] >= sent['lidar'], ('scan acknowledgement missing',sent,counts)
                if args.lidar_type == 1:
                    message=transport.Lidar();message.timebase=stamp;message.point_num=len(data)//19
                    points=[]
                    for values in struct.iter_unpack('<IfffBBB',data):
                        point=transport.Point()
                        point.offset_time,point.x,point.y,point.z,point.reflectivity,point.tag,point.line=values
                        points.append(point)
                    message.points=points
                else:
                    message=transport.Cloud();message.height=1;message.width=len(data)//19
                    schema=[('x',0,7),('y',4,7),('z',8,7),('intensity',12,7)]
                    if args.lidar_type == 2:
                        schema += [('time',16,7),('ring',20,4)];packing='<fffffH'
                    elif args.lidar_type == 3:
                        schema += [('t',16,6),('reflectivity',20,4),('ring',22,2),('ambient',23,4),('range',25,6)];packing='<ffffIHBHI'
                    else:
                        packing='<ffff'
                    message.fields=[transport.PointField(name=name,offset=offset,datatype=kind,count=1) for name,offset,kind in schema]
                    packed=[]
                    for _,x,y,z,reflectivity,_,line in struct.iter_unpack('<IfffBBB',data):
                        values=[x,y,z,float(reflectivity)]
                        if args.lidar_type == 2: values += [1e-9,line]
                        if args.lidar_type == 3: values += [1,reflectivity,line,0,int(math.sqrt(x*x+y*y+z*z)*1000)]
                        packed.append(struct.pack(packing,*values))
                    message.point_step=struct.calcsize(packing);message.row_step=message.point_step*message.width
                    message.is_dense=True;message.is_bigendian=False;message.data=b''.join(packed)
                transport.stamp(message,stamp);message.header.frame_id='livox_frame'
                sent['lidar']+=1;pending[sent['lidar']]=time.monotonic();transport.lidar.publish(message)
            transport.spin(0.)
            assert node.poll() is None, 'node exited during input'
        deadline=time.monotonic()+10
        while counts['ack'] < sent['lidar'] and time.monotonic() < deadline:
            transport.spin(.002)
        for _ in range(100):
            transport.spin(.01)
        duration=time.monotonic()-start
        stat=Path('/proc/{}/stat'.format(node.pid)).read_text().split()
        cpu=(int(stat[13])+int(stat[14]))/os.sysconf('SC_CLK_TCK')
        status=Path('/proc/{}/status'.format(node.pid)).read_text().splitlines()
        rss=int(next(line for line in status if line.startswith('VmHWM:')).split()[1])*1024
        for f in files:
            f.flush()
        assert not errors, errors[:4]
        assert counts['ack']==sent['lidar'], (sent,counts)
        assert counts['odom']==counts['body']==counts['world']==counts['gravity']==counts['tf'] and counts['odom']>10, counts
        assert counts['imu_odom']>counts['odom'] and counts['path']>0, counts
        result={'ros':args.ros,'lidar_type':args.lidar_type,'sent':sent,'received':counts,'wall_s':duration,'node_cpu_s':cpu-initial_cpu,'startup_cpu_s':initial_cpu,'mean_cores':(cpu-initial_cpu)/duration,'peak_rss_bytes':rss,'errors':errors}
        result['body_fields']=body_fields
        result['ack_observed_latency_s']=dict(zip(('p50','p95','max'),map(float,np.percentile(ack_latencies,[50,95,100]))))
    finally:
        for child in reversed(processes):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:
                    child.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGKILL);child.wait()
        if transport:
            transport.close()
        for f in files:
            f.close()
        stream.close()
    result['node_exit_code']=node.returncode
    assert node.returncode == 0, ('node shutdown failed', node.returncode)
    (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))

if __name__=='__main__':
    main()
