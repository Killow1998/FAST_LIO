#include "preprocess.h"

void Preprocess::avia_handler(const fast_lio::LivoxScan &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  double t1 = omp_get_wtime();
  int plsize = msg.points.size();

  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  for(int i=0; i<N_SCANS; i++)
  {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  }
  uint valid_num = 0;

  if (feature_enabled)
  {
    for(uint i=1; i<plsize; i++)
    {
      if((msg.points[i].line < N_SCANS) && ((msg.points[i].tag & 0x30) == 0x10 || (msg.points[i].tag & 0x30) == 0x00))
      {
        pl_full[i].x = msg.points[i].x;
        pl_full[i].y = msg.points[i].y;
        pl_full[i].z = msg.points[i].z;
        pl_full[i].intensity = msg.points[i].reflectivity;
        pl_full[i].curvature = msg.points[i].offset_time / float(1000000); //use curvature as time of each laser points

        bool is_new = false;
        if((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7)
            || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
            || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
        {
          pl_buff[msg.points[i].line].push_back(pl_full[i]);
        }
      }
    }
    extractFeatures();
  }
  else
  {
    for(uint i=1; i<plsize; i++)
    {
      if((msg.points[i].line < N_SCANS) && ((msg.points[i].tag & 0x30) == 0x10 || (msg.points[i].tag & 0x30) == 0x00))
      {
        valid_num ++;
        if (valid_num % point_filter_num == 0)
        {
          pl_full[i].x = msg.points[i].x;
          pl_full[i].y = msg.points[i].y;
          pl_full[i].z = msg.points[i].z;
          pl_full[i].intensity = msg.points[i].reflectivity;
          pl_full[i].curvature = msg.points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms

          if(((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7)
              || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
              || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))
              && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind * blind)))
          {
            pl_surf.push_back(pl_full[i]);
          }
        }
      }
    }
  }
}
