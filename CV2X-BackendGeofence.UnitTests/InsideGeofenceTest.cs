using System;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using CV2X_BackendGeofence;

namespace CV2X_BackendGeofence.UnitTests
{
    [TestClass]
    public class InsideGeofenceTest
    {
       
        [TestMethod]
        public void Geofence_InsidePedGOingTrafficIsland2_ReturnsTrue()
        {
            StaticWrapper wrapper = new StaticWrapper();
            wrapper.LoadXML();

            double lat = 1.3165859;
            double lon = 103.8625896;

            var res = new Geofence(wrapper.ptGoingTrafficIsland).IsInsideGeofence2(lat, lon);

            Assert.IsTrue(res);
        }

        [TestMethod]
        public void Geofence_InsidePedGOingTrafficIsland_ReturnsFalse()
        {
            StaticWrapper wrapper = new StaticWrapper();
            wrapper.LoadXML();

            double lat = 1.316617;
            double lon = 103.862597;

            var res = new Geofence(wrapper.ptGoingTrafficIsland).IsInsideGeofence2(lat, lon);

            Assert.IsFalse(res);
        }

        [TestMethod]
        public void Geofence_InsidePedGOingTrafficIsland2_ReturnsFalse()
        {
            StaticWrapper wrapper = new StaticWrapper();
            wrapper.LoadXML();

            double lat = 1.316637;
            double lon = 103.862674;

            var res = new Geofence(wrapper.ptGoingTrafficIsland).IsInsideGeofence2(lat, lon);

            Assert.IsFalse(res);
        }

        [TestMethod]
        public void Geofence_InsidePedTrafficIsland_ReturnsTrue()
        {
            StaticWrapper wrapper = new StaticWrapper();
            wrapper.LoadXML();

            double lat = 1.292072;
            double lon = 103.784199;

            var res = new Geofence(wrapper.ptTrafficIsland).IsInsideGeofence2(lat, lon);

            Assert.IsTrue(res);
        }

        [TestMethod]
        public void Geofence_InsidePedTrafficIsland2_ReturnsTrue()
        {
            StaticWrapper wrapper = new StaticWrapper();
            wrapper.LoadXML();

            double lat = 1.292036;
            double lon = 103.784183;

            var res = new Geofence(wrapper.ptTrafficIsland).IsInsideGeofence2(lat, lon);

            Assert.IsTrue(res);
        }

        [TestMethod]
        public void Geofence_CarTurningRight_ReturnsTrue()
        {
            StaticWrapper wrapper = new StaticWrapper();
            wrapper.LoadXML();

            double lat = 1.292065;
            double lon = 103.783965;

            var res = new Geofence(wrapper.ptCar_GoingRight).IsInsideGeofence2(lat, lon);

            Assert.IsTrue(res);
        }

        [TestMethod]
        public void Geofence_InsideCarGoingLeft_ReturnsTrue()
        {
            StaticWrapper wrapper = new StaticWrapper();
            wrapper.LoadXML();

            double lat = 1.291928;
            double lon = 103.784069;

            var res = new Geofence(wrapper.ptCar_GoingLeft).IsInsideGeofence2(lat, lon);

            Assert.IsTrue(res);
        }

        [TestMethod]
        public void Geofence_InsidePedGOingTrafficIsland_ReturnsTrue()
        {
            StaticWrapper wrapper = new StaticWrapper();
            wrapper.LoadXML();

            double lat = 1.292068;
            double lon = 103.784193;

            var res = new Geofence(wrapper.ptGoingTrafficIsland).IsInsideGeofence2(lat, lon);

            Assert.IsTrue(res);
        }
    }
}
