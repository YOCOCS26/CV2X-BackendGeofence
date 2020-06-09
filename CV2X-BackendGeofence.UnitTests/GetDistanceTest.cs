using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;

namespace CV2X_BackendGeofence.UnitTests
{
    [TestClass]
    public class GetDistanceTest
    {
        [TestMethod]
        public void GetDistance_InsideHotspot_ReturnsTrue()
        {
            CV2X_BackendGeofence.Program pg = new Program();
            double lat1 = 1.316880;
            double lon1 = 103.862559;
            double lat2 = 1.316665;
            double lon2 = 103.862713;
           
            var getDist = pg.getDistance(lat1, lon1, lat2, lon2);
            bool res = getDist < 200;

            Assert.IsTrue(res);
        }

        [TestMethod]
        public void GetDistance_InsideHotspot_2_ReturnsTrue()
        {
            CV2X_BackendGeofence.Program pg = new Program();
            double lat1 = 1.316880;
            double lon1 = 103.862559;
            double lat2 = 1.316661;
            double lon2 = 103.862571;
            var getDist = pg.getDistance(lat1, lon1, lat2, lon2);
            bool res = getDist < 200;

            Assert.IsTrue(res);
        }

        [TestMethod]
        public void GetDistance_OutsideHotspot_ReturnsFalse()
        {
            CV2X_BackendGeofence.Program pg = new Program();
            double lat1 = 1.316880;
            double lon1 = 103.862559;
            double lat2 = 1.318329;
            double lon2 = 103.863676;

            var getDist = pg.getDistance(lat1, lon1, lat2, lon2);
            bool res = getDist < 200;

            Assert.IsFalse(res);
        }

        [TestMethod]
        public void GetDistance_OutsideHotspot_2_ReturnsFalse()
        {
            CV2X_BackendGeofence.Program pg = new Program();
            double lat1 = 1.316880;
            double lon1 = 103.862559;
            double lat2 = 1.315464;
            double lon2 = 103.861428;

            var getDist = pg.getDistance(lat1, lon1, lat2, lon2);
            bool res = getDist < 200;

            Assert.IsFalse(res);
        }
    }
}
