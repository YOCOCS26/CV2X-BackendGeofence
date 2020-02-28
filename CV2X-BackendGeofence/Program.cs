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
    class Program
    {
        public static List<Point> ptHotspot = new List<Point>();
        public static List<Point> ptCar_GoingLeft = new List<Point>();
        public static List<Point> ptCar_GoingRight = new List<Point>();
        public static List<Point> ptCar_GoingStraight = new List<Point>();
        public static List<Point> ptCar_Yellowbox = new List<Point>();
        public static List<Point> ptGoingTrafficIsland = new List<Point>();
        public static List<Point> ptTrafficIsland = new List<Point>();
        public static List<Point> ptCrossing = new List<Point>();
        
        public static Geofence geofence;
        public static GPSData gpsData;

        /***************MQTT Broker Details*******************/
        public static MqttClient client;
        public static string host = "52.77.26.14";
        public static int port = 1883;
        public static string username = "admin";
        public static string password = "adminpassword";
        public static string[] m2mqtt_topic = new string[2];

        public const int radius = 200;

        public static GeoLocation vehicle;
        public static GeoLocation pedestrian;

        public static System.Timers.Timer tmr;

        public struct GeoLocation
        {
            public double Latitude { get; set; }
            public double Longitude { get; set; }
            public string Id { get; set; }
        }

        static void LoadXml(string filename, List<Point> listPts)
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

        static void InitMQTT()
        {
            try
            {
                client = new MqttClient(host);
                client.Settings.InflightQueueSize = 500;
                Console.WriteLine("Connecting to  MQTT Server," + host + "," + port + "....");
                client.Connect(Guid.NewGuid().ToString(), username, password);
                //client.MqttMsgPublishReceived += new MqttClient.MqttMsgPublishEventHandler(client_MqttMsgPublishReceived);
                client.MqttMsgPublishReceived += new MqttClient.MqttMsgPublishEventHandler(client_MqttMsgPublishReceived2);
                client.ConnectionClosed += new MqttClient.ConnectionClosedEventHandler(client_ConnectionClosed);
                if (client.IsConnected)
                {
                    Console.WriteLine("MQTT server connection is OK, " + host + ",  " + port);
                }
                else
                {
                    Console.WriteLine("MQTT server connection failed");
                }
                client.Subscribe(new string[] { "RTCU" }, new byte[] { MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE });
            }
            catch (Exception ex)
            {
                Console.WriteLine("Cannot connect to MQTT broker");
            }
        }

        private static void CheckCollision()
        {
            //check if both the pedestrian & the vehicle is inside the 200m radius Geofence
            if (getDistance(ptHotspot.First().X, ptHotspot.First().Y, vehicle.Latitude, vehicle.Longitude) < radius &&
                  getDistance(ptHotspot.First().X, ptHotspot.First().Y, pedestrian.Latitude, pedestrian.Longitude) < radius)
            {
                //load xml for car geofences
                LoadXml("Car_Going_Left.xml", ptCar_GoingLeft);
                LoadXml("Car_Going_Right.xml", ptCar_GoingRight);
                LoadXml("Car_Going_Straight.xml", ptCar_GoingStraight);
                LoadXml("Yellowbox.xml", ptCar_Yellowbox);

                //check if car is going left
                if (new Geofence(ptCar_GoingLeft).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude))
                {
                    //load geofence for pedestrian going to traffic island and inside traffic island
                    LoadXml("Pedestrian-1.xml", ptGoingTrafficIsland);
                    LoadXml("Traffic_Island.xml", ptTrafficIsland);

                    //check if pedestrian is inside the geofence going to traffic island
                    if (new Geofence(ptGoingTrafficIsland).IsInsideGeofence2(pedestrian.Latitude, pedestrian.Longitude))
                    {
                    }
                    //check if pedestrian is inside the traffic island
                    else if (new Geofence(ptTrafficIsland).IsInsideGeofence2(pedestrian.Latitude, pedestrian.Longitude))
                    {
                    }
                }
                //check if car is going right, going straight or inside yellowbox
                else if (new Geofence(ptCar_GoingRight).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude) ||
                        new Geofence(ptCar_GoingStraight).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude) ||
                        new Geofence(ptCar_Yellowbox).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude))
                {
                    //load the xml file for pedestrian inside the traffic island and pedestrian crossing geofence
                    LoadXml("Traffic_Island.xml", ptTrafficIsland);
                    LoadXml("Pedestrian_Crossing.xml", ptCrossing);

                    //check if car is going right & pedestrian is inside traffic island
                    if (new Geofence(ptCar_GoingRight).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude) &&
                        new Geofence(ptTrafficIsland).IsInsideGeofence2(pedestrian.Latitude, pedestrian.Longitude))
                    {
                    }
                    //check if car is going right & pedestrian is crossing
                    else if (new Geofence(ptCar_GoingRight).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude) &&
                        new Geofence(ptCrossing).IsInsideGeofence2(pedestrian.Latitude, pedestrian.Longitude))
                    {
                    }
                    //check if car is going striaght & pedestrian is inside traffic island
                    else if (new Geofence(ptCar_GoingStraight).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude) &&
                        new Geofence(ptTrafficIsland).IsInsideGeofence2(pedestrian.Latitude, pedestrian.Longitude))
                    {
                    }
                    //check if car is going straight & pedestrian is crossing
                    else if (new Geofence(ptCar_GoingStraight).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude) &&
                        new Geofence(ptCrossing).IsInsideGeofence2(pedestrian.Latitude, pedestrian.Longitude))
                    {
                    }
                    //check if car is inside yellowbox & pedestrian is inside traffic island
                    else if (new Geofence(ptCar_Yellowbox).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude) &&
                        new Geofence(ptTrafficIsland).IsInsideGeofence2(pedestrian.Latitude, pedestrian.Longitude))
                    {
                    }
                    //check if car is inside yellowbox & pedestrian is crossing
                    else if (new Geofence(ptCar_Yellowbox).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude) &&
                        new Geofence(ptCrossing).IsInsideGeofence2(pedestrian.Latitude, pedestrian.Longitude))
                    {
                    }
                }
            }          
        }

        private static void client_MqttMsgPublishReceived(object sender, MqttMsgPublishEventArgs e)
        {
            if (String.Equals(e.Topic, "RTCU"))
            {
                try
                {
                    gpsData = GPSData.Parser.ParseFrom(e.Message);

                    if (gpsData.VehicleType == 0)       //vehicle
                    {
                        vehicle.Latitude = gpsData.Latitude;
                        vehicle.Longitude = gpsData.Longitude;
                        vehicle.Id = gpsData.Id;

                        //check if vehicle is inside the 200m radius Geofence
                        if (getDistance(ptHotspot[0].X, ptHotspot[0].Y, vehicle.Latitude, vehicle.Longitude) < radius)
                        {
                            //load xml for vehicle geofences
                            LoadXml("Car_Going_Left.xml", ptCar_GoingLeft);
                            LoadXml("Car_Going_Right.xml", ptCar_GoingRight);
                            LoadXml("Car_Going_Straight.xml", ptCar_GoingStraight);
                            LoadXml("Yellowbox.xml", ptCar_Yellowbox);

                            //ckeck if car is going left
                            if (new Geofence(ptCar_GoingLeft).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude))
                            {
                                //load the xml file for pedestrian going to traffic island and inside the traffic island geofence
                                LoadXml("Pedestrian-1.xml", ptGoingTrafficIsland);
                                LoadXml("Traffic_Island.xml", ptTrafficIsland);

                                //check if pedestrian is inside the geofence going to traffic island
                                if (new Geofence(ptGoingTrafficIsland).IsInsideGeofence2(pedestrian.Latitude, pedestrian.Longitude))
                                {
                                }
                                //check if pedestrian is inside the traffic island
                                else if (new Geofence(ptTrafficIsland).IsInsideGeofence2(pedestrian.Latitude, pedestrian.Longitude))
                                {
                                }
                            }

                            //check if car is going right, going straight or inside yellowbox
                            else if ((new Geofence(ptCar_GoingRight).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude))
                                     || (new Geofence(ptCar_GoingStraight).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude))
                                     || (new Geofence(ptCar_Yellowbox).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude)))
                            {
                                //load the xml file for pedestrian inside the traffic island and pedestrian crossing geofence
                                LoadXml("Traffic_Island.xml", ptTrafficIsland);
                                LoadXml("Pedestrian_Crossing.xml", ptCrossing);

                                //check if vehicle is going right & pedestrian is inside the traffic island
                                if (new Geofence(ptCar_GoingRight).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude) &&
                                    new Geofence(ptTrafficIsland).IsInsideGeofence2(pedestrian.Latitude, pedestrian.Longitude))
                                {
                                }
                                //check if vehicle is going right & pedestrian is crossing
                                else if (new Geofence(ptCar_GoingRight).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude) &&
                                    new Geofence(ptCrossing).IsInsideGeofence2(pedestrian.Latitude, pedestrian.Longitude))
                                {
                                }
                                //check if vehicle is going straight & pedestrian is inside the traffic island
                                else if (new Geofence(ptCar_GoingStraight).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude) &&
                                    new Geofence(ptTrafficIsland).IsInsideGeofence2(pedestrian.Latitude, pedestrian.Longitude))
                                {
                                }
                                //check if vehicle is going straight & pedestrian is crossing
                                else if (new Geofence(ptCar_GoingStraight).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude) &&
                                    new Geofence(ptCrossing).IsInsideGeofence2(pedestrian.Latitude, pedestrian.Longitude))
                                {
                                }
                                //check if vehicle is inside yellowbox & pedestrian is inside the traffic island
                                else if (new Geofence(ptCar_Yellowbox).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude) &&
                                    new Geofence(ptTrafficIsland).IsInsideGeofence2(pedestrian.Latitude, pedestrian.Longitude))
                                {
                                }
                                //check if vehicle is inside yellowbox & pedestrian is crossing
                                else if (new Geofence(ptCar_Yellowbox).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude) &&
                                    new Geofence(ptCrossing).IsInsideGeofence2(pedestrian.Latitude, pedestrian.Longitude))
                                {
                                }
                            }
                        }
                    }

                    else if (gpsData.VehicleType == 1)       //pedestrian
                    {
                        pedestrian.Latitude = gpsData.Latitude;
                        pedestrian.Longitude = gpsData.Longitude;
                        pedestrian.Id = gpsData.Id;                       

                        if (getDistance(ptHotspot[0].X, ptHotspot[0].Y, pedestrian.Latitude, pedestrian.Longitude) < radius)
                        {
                            //load xml for pedestrian geofences
                            LoadXml("Pedestrian-1.xml", ptGoingTrafficIsland);
                            LoadXml("Pedestrian_Crossing.xml", ptCrossing);
                            LoadXml("Traffic_Island.xml", ptTrafficIsland);

                            //check if the pedestrian in inside the geofence going to traffic island
                            if (new Geofence(ptGoingTrafficIsland).IsInsideGeofence2(pedestrian.Latitude, pedestrian.Longitude))
                            {
                                //load car going left geofence
                                LoadXml("Car_Going_Left.xml", ptCar_GoingLeft);

                                //check if car is inside the geofence going left
                                if (new Geofence(ptCar_GoingLeft).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude))
                                {
                                }
                            }
                            //check if pedestrian is inside the pedestrian crossing geofence
                            else if (new Geofence(ptCrossing).IsInsideGeofence2(pedestrian.Latitude, pedestrian.Longitude))
                            {
                                //load car geofences for cars going right, going straight, & inside yellowbox
                                LoadXml("Car_Going_Right.xml", ptCar_GoingRight);
                                LoadXml("Car_Going_Straight.xml", ptCar_GoingStraight);
                                LoadXml("Yellowbox.xml", ptCar_Yellowbox);

                                //check if car is inside going right geofence
                                if (new Geofence(ptCar_GoingRight).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude))
                                {
                                }
                                //check if car is inside the going straight geofence
                                else if (new Geofence(ptCar_GoingStraight).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude))
                                {
                                }
                                //check if car is inside yellowbox
                                else if (new Geofence(ptCar_Yellowbox).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude))
                                {
                                }
                            }
                            //check if pedestrian is inside the traffic island
                            else if (new Geofence(ptTrafficIsland).IsInsideGeofence2(pedestrian.Latitude, pedestrian.Longitude))
                            {
                                //load car geofences for cars going left, going right, going straight, & inside yellowbox
                                LoadXml("Car_Going_Left.xml", ptCar_GoingLeft);
                                LoadXml("Car_Going_Right.xml", ptCar_GoingRight);
                                LoadXml("Car_Going_Straight.xml", ptCar_GoingStraight);
                                LoadXml("Yellowbox.xml", ptCar_Yellowbox);

                                //check if car is inside going left geofence                            
                                if (new Geofence(ptCar_GoingLeft).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude))
                                {
                                }
                                //check if car is inside going right geofence
                                else if (new Geofence(ptCar_GoingRight).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude))
                                {
                                }
                                //check if car is inside the going straight geofence
                                else if (new Geofence(ptCar_GoingStraight).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude))
                                {
                                }
                                //check if car is inside yellowbox
                                else if (new Geofence(ptCar_Yellowbox).IsInsideGeofence2(vehicle.Latitude, vehicle.Longitude))
                                {
                                }
                            }
                        }                      
                    }                   
                    var msg = "IN";
                    var mClient = sender as MqttClient;

                    mClient.Publish("geofence", Encoding.ASCII.GetBytes(msg));
                }
                catch (Exception ex)
                {
                    Console.WriteLine(ex.Message);
                }
            }
        }

        private static void client_MqttMsgPublishReceived2(object sender, MqttMsgPublishEventArgs e)
        {
            if (String.Equals(e.Topic, "RTCU"))
            {
                gpsData = GPSData.Parser.ParseFrom(e.Message);

                if (gpsData.VehicleType == 0)
                {
                    vehicle.Latitude = gpsData.Latitude;
                    vehicle.Longitude = gpsData.Longitude;
                    vehicle.Id = gpsData.Id;
                }
                else if (gpsData.VehicleType == 1)
                {
                    pedestrian.Latitude = gpsData.Latitude;
                    pedestrian.Longitude = gpsData.Longitude;
                    pedestrian.Id = gpsData.Id;
                }
            }
        }

        private static void client_ConnectionClosed(object sender, EventArgs e)
        {
            Console.WriteLine("Disconnected to MQTT Broker");
            CancellationTokenSource source = new CancellationTokenSource();
            Task t = Task.Run(() => TryReconnectAsync(source.Token));
        }

        private static async Task TryReconnectAsync(CancellationToken cancellationToken)
        {
            var connected = client.IsConnected;
            while (!connected && !cancellationToken.IsCancellationRequested)
            {
                try
                {
                    client.Connect(Guid.NewGuid().ToString(), username, password);
                }
                catch
                {
                    Console.WriteLine("Failed to connect to MQTT Broker");
                }
                connected = client.IsConnected;
                await Task.Delay(1000, cancellationToken);
            }
            client.Subscribe(new string[] { "RTCU" }, new byte[] { MqttMsgBase.QOS_LEVEL_AT_MOST_ONCE });
        }

        static void InitProtobuf()
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

            vehicle = new GeoLocation();
            vehicle.Latitude = 0;
            vehicle.Longitude = 0;
            vehicle.Id = "";
            pedestrian = new GeoLocation();
            pedestrian.Latitude = 0;
            pedestrian.Longitude = 0;
            pedestrian.Id = "";
        }

        static void InitTimer()
        {
            tmr = new System.Timers.Timer(1000);
            tmr.AutoReset = true;
            tmr.Elapsed += Tmr_Elapsed;
            tmr.Enabled = true;
        }

        private static void Tmr_Elapsed(object sender, System.Timers.ElapsedEventArgs e)
        {
            try
            {
                CheckCollision();
            }
            catch
            {
                throw new NotImplementedException();
            }
        }

        public static double getDistance(double lat1, double lon1, double lat2, double lon2)
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

        public static double ToRad(double deg)
        {
            return deg * (Math.PI / 180);
        }

        static void Main(string[] args)
        {
            LoadXml("Hotspot.xml", ptHotspot);
            InitProtobuf();
            InitMQTT();
            InitTimer();

            for (;;)
            {
            }
        }
    }
}
