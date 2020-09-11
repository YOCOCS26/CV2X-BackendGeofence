using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Data;
using System.Threading;

using uPLibrary.Networking.M2Mqtt.Messages;
using uPLibrary.Networking.M2Mqtt;
using Google.Protobuf;

namespace CV2X_BackendGeofence
{
    public class StaticWrapper
    {
        public List<Point> ptHotspot = new List<Point>();
        public List<Point> ptCar_GoingLeft = new List<Point>();
        public List<Point> ptCar_GoingRight = new List<Point>();
        public List<Point> ptCar_GoingStraight = new List<Point>();
        public List<Point> ptGoingTrafficIsland = new List<Point>();
        public List<Point> ptTrafficIsland = new List<Point>();
        public List<Point> ptCrossing = new List<Point>();
        public List<Point> ptTruck_GoingStraight = new List<Point>();

        public void LoadXML()
        {
            Program.LoadXml("Science Park/Car_Going_Left.xml", out ptCar_GoingLeft);
            Program.LoadXml("Science Park/Car_Going_Right.xml", out ptCar_GoingRight);
            Program.LoadXml("Science Park/Car_Going_Straight.xml", out ptCar_GoingStraight);
            Program.LoadXml("Science Park/Ped_Going_TrafficIsland.xml", out ptGoingTrafficIsland);
            Program.LoadXml("Science Park/Ped_Traffic_Island.xml", out ptTrafficIsland);
            Program.LoadXml("Science Park/Ped_Crossing.xml", out ptCrossing);
            Program.LoadXml("Science Park/Truck_Going_Straight.xml", out ptTruck_GoingStraight);
        }
    }

    public class Program
    {
        public static List<Point> ptHotspot = new List<Point>();
        public static List<Point> ptCar_GoingLeft = new List<Point>();
        public static List<Point> ptCar_GoingRight = new List<Point>();
        public static List<Point> ptCar_GoingStraight = new List<Point>();
        public static List<Point> ptGoingTrafficIsland = new List<Point>();
        public static List<Point> ptTrafficIsland = new List<Point>();
        public static List<Point> ptCrossing = new List<Point>();
        public static List<Point> ptTruck_GoingStraight = new List<Point>();

        public static List<Point> ptBusStop = new List<Point>();
        public static List<int> iBusStopId = new List<int>();

        static Geofence geofence;
        public static GPSData gpsData;
        public static CollisionData collisionData;
        public static CollisionDetails collisionDetailsV2X;
        public static DispatchData dispatchDetails;

        /***************MQTT Broker Details*******************/
        public static MqttClient client;
        public static string host = "52.77.26.14";
        public static int port = 1883;
        public static string username = "admin";
        public static string password = "adminpassword";
        public static string[] m2mqtt_topic = new string[2];

        /***************MQTT Publish Details******************/
        public static string topic = "cv2x";
        public static string collisionTopic = "VehicleCollisionData";
        public static string dispatchTopic = "DispatchResult";

        public const int radius = 200;

        public static GeoLocation vehicle;
        public static GeoLocation pedestrian;

        public static List<BusStopDetails> busStopDetails = new List<BusStopDetails>();

        public static System.Timers.Timer tmr;
        public static System.Timers.Timer tmrDispatch;
        public static int loopCntr = 0;
        public static int mqttCarCntr = 0;
        public static int mqttPedCntr = 0;

        public struct GeoLocation
        {
            public double Latitude { get; set; }
            public double Longitude { get; set; }
            public string Id { get; set; }
        }

        public class BusStopDetails
        {
            public int counter { get; set; }
            public int reset { get; set; }
        }

        public static void LoadXml(string filename, out List<Point> listPts)
        {
            DataSet ds = new DataSet();
            ds.ReadXml(filename);

            listPts = new List<Point>();

            foreach (DataRow dr in ds.Tables[0].Rows)
            {
                Point p = new Point();

                String lat = dr[0].ToString();
                p.X = Double.Parse(lat);

                String lon = dr[1].ToString();
                p.Y = Double.Parse(lon);

                listPts.Add(p);
            }
        }

        public static void LoadBusStop(string filename, out List<Point> listBusStop, out List<int> busstopId)
        {
            DataSet ds = new DataSet();
            ds.ReadXml(filename);

            listBusStop = new List<Point>();
            busstopId = new List<int>();

            foreach (DataRow dr in ds.Tables[0].Rows)
            {
                Point p = new Point();

                String lat = dr[0].ToString();
                p.X = Double.Parse(lat);

                String lon = dr[1].ToString();
                p.Y = Double.Parse(lon);

                listBusStop.Add(p);

                int id = Int32.Parse(dr[2].ToString());
                busstopId.Add(id);
            }
        }

        public void InitMQTT()
        {
            try
            {
                client = new MqttClient(host);
                client.Settings.InflightQueueSize = 500;
                Console.WriteLine("Connecting to  MQTT Server," + host + ":" + port + "....");
                client.Connect(Guid.NewGuid().ToString(), username, password);
                client.MqttMsgPublishReceived += new MqttClient.MqttMsgPublishEventHandler(client_MqttMsgPublishReceived);
                client.ConnectionClosed += new MqttClient.ConnectionClosedEventHandler(client_ConnectionClosed);
                if (client.IsConnected)
                {
                    Console.WriteLine("MQTT server connection is OK, " + host + ",  " + port);
                    client.Subscribe(new string[] { "RTCU" }, new byte[] { MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE });
                }
                else
                {
                    Console.WriteLine("MQTT server connection failed");
                }
            }
            catch (Exception ex)
            {
                Thread.Sleep(1000);
                Console.WriteLine("Cannot connect to MQTT broker");
                CancellationTokenSource source = new CancellationTokenSource();
                Task t = Task.Run(() => TryReconnectAsync(source.Token));
            }
        }

        public void CheckCollisionDetails()
        {
            double collisionDistance = getDistance(vehicle.Latitude, vehicle.Longitude, pedestrian.Latitude, pedestrian.Longitude);

            collisionDetailsV2X.AlertType = 1;
            collisionDetailsV2X.CollisionDetectionFlag = 0;
            collisionDetailsV2X.TargetVehicleId = pedestrian.Id;
            collisionDetailsV2X.SourceVehicleId = vehicle.Id;
            collisionDetailsV2X.DistanceToCollision = collisionDistance;
            collisionDetailsV2X.CollisionLatitude = pedestrian.Latitude;
            collisionDetailsV2X.CollisionLongitude = pedestrian.Longitude;

            if (collisionData.CollisionStatus != 0)
            {
                if (collisionDistance <= collisionDetailsV2X.AlertBRAKERadius)
                {
                    collisionDetailsV2X.CollisionDetectionFlag = 1;
                    collisionDetailsV2X.AlertType = 4;
                    client.Publish("seathaptic", Encoding.UTF8.GetBytes("ON"), 0, false);
                }
                else if (collisionDistance <= collisionDetailsV2X.AlertWARNRadius)
                {
                    collisionDetailsV2X.CollisionDetectionFlag = 1;
                    collisionDetailsV2X.AlertType = 3;
                    client.Publish("seathaptic", Encoding.UTF8.GetBytes("ON"), 0, false);
                }
                else if (collisionDistance <= collisionDetailsV2X.AlertINFORadius)
                {
                    collisionDetailsV2X.CollisionDetectionFlag = 1;
                    collisionDetailsV2X.AlertType = 2;
                    client.Publish("seathaptic", Encoding.UTF8.GetBytes("OFF"), 0, false);
                }
            }
            else
            {
                client.Publish("seathaptic", Encoding.UTF8.GetBytes("OFF"), 0, false);
            }    

            client.Publish(collisionTopic, collisionDetailsV2X.ToByteArray(), (byte)0, false);            
        }

        public void CheckCollision()
        {
            //check if both the pedestrian & the vehicle is inside the 200m radius Hotspot Zone
            if ((getDistance(ptHotspot.First().X, ptHotspot.First().Y, vehicle.Latitude, vehicle.Longitude) < radius) &&
                  (getDistance(ptHotspot.First().X, ptHotspot.First().Y, pedestrian.Latitude, pedestrian.Longitude) < radius))
            {
                //load xml for car geofences
                LoadXml("Science Park/Car_Going_Left.xml", out ptCar_GoingLeft);
                LoadXml("Science Park/Car_Going_Right.xml", out ptCar_GoingRight);
                LoadXml("Science Park/Car_Going_Straight.xml", out ptCar_GoingStraight);
                LoadXml("Science Park/Truck_Going_Straight.xml", out ptTruck_GoingStraight);         

                //check if car is turning right (Usecase 1 & 2)
                if (new Geofence(ptCar_GoingRight).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude))
                {
                    //load geofence for pedestrian crossing between two traffic islands
                    LoadXml("Science Park/Ped_Traffic_Island.xml", out ptTrafficIsland);

                    //check if pedestrian is inside the geofence between two traffic island (Usecase 1)
                    if (new Geofence(ptTrafficIsland).IsInsideGeofence2(pedestrian.Latitude, pedestrian.Longitude))
                    {
                        collisionData.CollisionStatus = 1;
                    }
                    //check if truck is going straight while car is turning right (Usecase 2)
                    else if (new Geofence(ptCar_GoingStraight).IsInsideGeofence2(pedestrian.Latitude, pedestrian.Longitude))
                    {
                        collisionData.CollisionStatus = 2;
                    }                
                }

                //check if car is going straight (Usecase 3)
                else if (new Geofence(ptCar_GoingStraight).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude))
                {
                    //load the xml file for pedestrian crossing geofence
                    LoadXml("Science Park/Ped_Crossing.xml", out ptCrossing);

                    //check if car is going straight & pedestrian is crossing
                    if (new Geofence(ptCar_GoingStraight).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude) &&
                        new Geofence(ptCrossing).IsInsideGeofence2(pedestrian.Latitude, pedestrian.Longitude))
                    {
                        collisionData.CollisionStatus = 3;
                    }                
                }
                //check if Car it turning Left (Usecase 4 & 5)
                else if (new Geofence(ptCar_GoingLeft).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude))
                {
                    //load the xml file for pedestrian going to traffic island geofence
                    LoadXml("Science Park/Ped_Going_TrafficIsland.xml", out ptGoingTrafficIsland);

                    //Check if car is turning left & pedestrian is crossing to traffic island (Usecase 4)
                    if (new Geofence(ptCar_GoingLeft).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude) &&
                        new Geofence(ptGoingTrafficIsland).IsInsideGeofence2(pedestrian.Latitude, pedestrian.Longitude))
                    {
                        collisionData.CollisionStatus = 4;
                    }

                    //Check if car is turning left & truck is approaching (Usecase 5)
                    if (new Geofence(ptCar_GoingLeft).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude) &&
                        new Geofence(ptTruck_GoingStraight).IsInsideGeofence2(pedestrian.Latitude, pedestrian.Longitude))
                    {
                        collisionData.CollisionStatus = 5;
                    }
                }

                else
                {
                    collisionData.CollisionStatus = 0;
                }
            }
            else
            {
                collisionData.CollisionStatus = 0;               
            }

            CheckCollisionDetails();       

            collisionData.VehicleId = vehicle.Id;
            collisionData.PedestrianId = pedestrian.Id;
            collisionData.Distance = getDistance(vehicle.Latitude, vehicle.Longitude, pedestrian.Latitude, pedestrian.Longitude);
            client.Publish(topic, collisionData.ToByteArray(), (byte)0, false);            
        }

        private void client_MqttMsgPublishReceived(object sender, MqttMsgPublishEventArgs e)
        {
            if (String.Equals(e.Topic, "RTCU"))
            {
                gpsData = GPSData.Parser.ParseFrom(e.Message);

                if (gpsData.Id == "car")
                {
                    vehicle.Latitude = gpsData.Latitude;
                    vehicle.Longitude = gpsData.Longitude;
                    vehicle.Id = gpsData.Id;
                    mqttCarCntr = 0;
                }
                else if (gpsData.Id == "pedestrian" || gpsData.VehicleType == 4)
                {
                    pedestrian.Latitude = gpsData.Latitude;
                    pedestrian.Longitude = gpsData.Longitude;
                    pedestrian.Id = gpsData.Id;
                    mqttPedCntr = 0;
                }

                if (vehicle.Id != null && pedestrian.Id != null)
                {
                    CheckCollision();
                    Console.WriteLine("[" + vehicle.Id + "]: lat=" + vehicle.Latitude + ", lon=" + vehicle.Longitude);
                    Console.WriteLine("[" + pedestrian.Id + "]: lat=" + pedestrian.Latitude + ", lon=" + pedestrian.Longitude);
                    Console.WriteLine("Collision Details-- Vehicle ID: " + vehicle.Id + ", Pedestrian ID: " + pedestrian.Id + ", Collision Status: " + collisionData.CollisionStatus + ", Distance: " + collisionData.Distance);
                    vehicle.Id = null;
                    pedestrian.Id = null;
                    loopCntr = 0;
                }
            }
        }

        private void client_ConnectionClosed(object sender, EventArgs e)
        {
            Console.WriteLine("Disconnected to MQTT Broker");
            CancellationTokenSource source = new CancellationTokenSource();
            Task t = Task.Run(() => TryReconnectAsync(source.Token));
        }

        private async Task TryReconnectAsync(CancellationToken cancellationToken)
        {
            var connected = client.IsConnected;
            while (!connected && !cancellationToken.IsCancellationRequested)
            {
                Console.WriteLine("Trying to connect to MQTT Broker");
                try
                {
                    client.Connect(Guid.NewGuid().ToString(), username, password);
                    client.MqttMsgPublishReceived += new MqttClient.MqttMsgPublishEventHandler(client_MqttMsgPublishReceived);
                    client.ConnectionClosed += new MqttClient.ConnectionClosedEventHandler(client_ConnectionClosed);
                    if (client.IsConnected)
                    {
                        Console.WriteLine("MQTT server connection is OK, " + host + ",  " + port);
                        client.Subscribe(new string[] { "RTCU" }, new byte[] { MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE });
                    }
                }
                catch
                {
                    Console.WriteLine("Failed to connect to MQTT Broker");
                }
                connected = client.IsConnected;
                await Task.Delay(1000, cancellationToken);
            }
        }

        public void InitProtobuf()
        {
            gpsData = new GPSData();
            gpsData.Timestamp = 0;
            gpsData.Id = "";
            gpsData.VehicleType = 0;
            gpsData.Latitude = 0;
            gpsData.Longitude = 0;
            gpsData.Bearing = 0;
            gpsData.Speed = 0;
            gpsData.Accuracy = 0;
            gpsData.Altitude = 0;
            gpsData.Provider = "";
            gpsData.SatellitesUsed = 0;
            gpsData.SatellitesInView = 0;
            gpsData.Imei = "";
            gpsData.Ssid = "";

            collisionData = new CollisionData();
            collisionData.VehicleId = "";
            collisionData.PedestrianId = "";
            collisionData.CollisionStatus = 0;
            collisionData.Distance = 0;

            collisionDetailsV2X = new CollisionDetails();
            collisionDetailsV2X.AbsCollisionTime = 0;
            collisionDetailsV2X.AlertBRAKERadius = 10;
            collisionDetailsV2X.AlertWARNRadius = 20;
            collisionDetailsV2X.AlertINFORadius = 30;
            collisionDetailsV2X.AlertType = 0;
            collisionDetailsV2X.CollisionAltitude = 0;
            collisionDetailsV2X.CollisionDetectionFlag = 0;
            collisionDetailsV2X.CollisionLatitude = 0;
            collisionDetailsV2X.CollisionLongitude = 0;
            collisionDetailsV2X.DistanceToCollision = 0;
            collisionDetailsV2X.SourceVehicleId = "";
            collisionDetailsV2X.TargetVehicleId = "";

            dispatchDetails = new DispatchData();
            dispatchDetails.BusstopId = "60011";
            dispatchDetails.BusstopLat = ptBusStop.First().X;
            dispatchDetails.BusstopLon = ptBusStop.First().Y;
            dispatchDetails.ZoneRadius = 15;
            dispatchDetails.TimeInsideZone = 0;
            dispatchDetails.TimeOutsideZone = 10;
            dispatchDetails.BusNumber = 0;
            dispatchDetails.EstArrivalTime = 0;
            dispatchDetails.NumOfCommuters = 0;
            dispatchDetails.TimerThreshold = 30;
            dispatchDetails.CommuterThreshold = 1;
            dispatchDetails.IsInsideZone = false;

            vehicle = new GeoLocation();
            vehicle.Latitude = 0;
            vehicle.Longitude = 0;
            vehicle.Id = null;
            pedestrian = new GeoLocation();
            pedestrian.Latitude = 0;
            pedestrian.Longitude = 0;
            pedestrian.Id = null;

            for (int i = 0; i < iBusStopId.Count; i++)
            {              
                busStopDetails.Add(new BusStopDetails
                {
                    reset = 0,
                    counter = 0
                });         
            }          
        }

        public void InitTimer()
        {
            tmr = new System.Timers.Timer(1000);
            tmr.AutoReset = true;
            tmr.Elapsed += Tmr_Elapsed;
            tmr.Enabled = true;

            tmrDispatch = new System.Timers.Timer(1000);
            tmrDispatch.AutoReset = true;
            tmrDispatch.Elapsed += tmrDispatch_Elapsed;
            tmrDispatch.Enabled = true;
        }

        private void tmrDispatch_Elapsed(object sender, System.Timers.ElapsedEventArgs e)
        {
            if (pedestrian.Id != null)
            {
                for (int i = 0; i < iBusStopId.Count - 1; i++)
                {
                    if (getDistance(ptBusStop[i].X, ptBusStop[i].Y, pedestrian.Latitude, pedestrian.Longitude) < dispatchDetails.ZoneRadius)
                    {
                        dispatchDetails.BusstopId = iBusStopId[i].ToString();
                        dispatchDetails.BusstopLat = ptBusStop[i].X;
                        dispatchDetails.BusstopLon = ptBusStop[i].Y;
                        busStopDetails[i].counter++;
                        busStopDetails[i].reset = 0;                       

                        if (busStopDetails[i].counter >= dispatchDetails.TimerThreshold)
                        {
                            busStopDetails[i].counter = dispatchDetails.TimerThreshold;
                            dispatchDetails.NumOfCommuters = 1;
                            dispatchDetails.IsInsideZone = true;
                        }
                    }
                    else
                    {
                        busStopDetails[i].reset++;

                        if (busStopDetails[i].reset > dispatchDetails.TimeOutsideZone)
                        {
                            busStopDetails[i].counter = 0;
                            dispatchDetails.IsInsideZone = false;
                            dispatchDetails.NumOfCommuters = 0;
                        }
                    }
                    dispatchDetails.TimeInsideZone = busStopDetails[i].counter;
                    client.Publish(dispatchTopic, dispatchDetails.ToByteArray(), (byte)0, false);
                }
            }
        }

        private void Tmr_Elapsed(object sender, System.Timers.ElapsedEventArgs e)
        {
            try
            {
                mqttPedCntr++;
                mqttCarCntr++;

                if (mqttCarCntr > 5)
                {
                    Console.WriteLine("Loop: " + loopCntr++ + " ------waiting for Car data from MQTT------");
                }
                if (mqttPedCntr > 5)
                {
                    Console.WriteLine("Loop: " + loopCntr++ + " ------waiting for pedestrian data from MQTT------");
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.Message);
                //throw new NotImplementedException();
            }
        }

        public double getDistance(double lat1, double lon1, double lat2, double lon2)
        {
            var R = 6371;
            var dLat = ToRad(lat2 - lat1);
            var dLon = ToRad(lon2 - lon1);

            var a = Math.Sin(dLat / 2) * Math.Sin(dLat / 2) + Math.Cos(ToRad(lat1)) *
                    Math.Cos(ToRad(lat2)) * Math.Sin(dLon / 2) * Math.Sin(dLon / 2);
            var c = 2 * Math.Atan2(Math.Sqrt(a), Math.Sqrt(1 - a));
            var d = R * c * 1000;  //convert to meter

            return d;
        }

        public double ToRad(double deg)
        {
            return deg * (Math.PI / 180);
        }

        static void Main(string[] args)
        {
            LoadXml("Science Park/Hotspot.xml", out ptHotspot);
            LoadBusStop("BusStop.xml", out ptBusStop, out iBusStopId);

            Program p = new Program();
            p.InitProtobuf();
            p.InitMQTT();
            while (!client.IsConnected) ;
            p.InitTimer();

            for (;;)
            {
            }
        }
    }
}
