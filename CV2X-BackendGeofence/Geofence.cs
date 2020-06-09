using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CV2X_BackendGeofence
{
    public class Point
    {
        public double X;
        public double Y;
    }

    public class Geofence
    {
        public List<Point> geofencePts = new List<Point>();

        public Geofence()
        {
        }

        public Geofence(List<Point> points)
        {
            foreach (Point p in points)
            {
                this.geofencePts.Add(p);
            }
        }

        public void Add(Point p)
        {
            this.geofencePts.Add(p);
        }

        public int Count()
        {
            return geofencePts.Count;
        }

        public bool IsInsideGeofence(double lat, double lon)
        {
            int sides = this.Count() - 1;
            int j = sides - 1;
            bool ptStatus = false;
            for (int i = 0; i < sides; i++)
            {
                if (geofencePts[i].Y < lon && geofencePts[j].Y >= lon || geofencePts[j].Y < lon && geofencePts[i].Y >= lon)
                {
                    if (geofencePts[i].X + (lon - geofencePts[i].Y) / (geofencePts[j].Y - geofencePts[i].Y) * (geofencePts[j].X - geofencePts[i].X) < lat)
                    {
                        ptStatus = !ptStatus;
                    }
                }
                j = i;
            }
            return ptStatus;
        }

        public bool IsInsideGeofence2(double lat, double lon)
        {
            int i, j;
            bool ptStatus = false;
            for (i = 0, j = geofencePts.Count - 1; i < geofencePts.Count; j = i++)
            {
                if ((((geofencePts[i].X <= lat) && (lat < geofencePts[j].X)) || 
                    ((geofencePts[j].X <= lat) && (lat < geofencePts[i].X))) && 
                    (lon < (geofencePts[j].Y - geofencePts[i].Y) * (lat - geofencePts[i].X) / (geofencePts[j].X - geofencePts[i].X) + geofencePts[i].Y))
                {
                    ptStatus = !ptStatus;
                }
            }
            return ptStatus;
        }
    }
}
