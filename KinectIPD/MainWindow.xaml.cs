using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;


namespace KinectIPD
{
    /// <summary>
    /// MainWindow.xaml 的交互逻辑
    /// </summary>
    public partial class MainWindow : Window
    {
        //The Kinect Indoor Pedestrian Detection
        
        const float MaxDepthDistance = 4000;// max value returned
        const float MinDepthDistance = 850;// min value returned
        const float MaxDepthDistanceOffset = MaxDepthDistance - MinDepthDistance;
        KinectSensor _sensor;
        Skeleton[] skData;
        Brush[] skBrushes = new Brush[6];
        bool haveSkeletonData;
        int flag = 1;
        
        Dictionary<JointType, Point> jointMapping = new Dictionary<JointType, Point>();
        PersonWPF[] Persons = new PersonWPF[6];

        //private WriteableBitmap outputBitmap;
        //bool haveNewFormat;
        private static readonly int Bgr32BytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;

        public MainWindow()
        {
            InitializeComponent();
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            //kinectSensorChooser1.KinectSensorChanged += new DependencyPropertyChangedEventHandler(kinectSensorChooser_KinectSensorChanged);
            //kinectSensorChooser1.KinectSensorChooserLoaded(null, null);
            
            if (KinectSensor.KinectSensors.Count > 0)
            {
                _sensor = KinectSensor.KinectSensors.FirstOrDefault();
                _sensor.ColorStream.Enable();
                _sensor.DepthStream.Enable();
                _sensor.SkeletonStream.Enable();
                _sensor.AllFramesReady += new EventHandler<AllFramesReadyEventArgs>(sensor_AllFramesReady);
                kinectColorViewer1.Kinect = _sensor;
                try
                {
                    _sensor.Start();
                }
                catch (System.IO.IOException)
                {
                    kinectSensorChooser1.AppConflictOccurred();
                }
                _sensor.ElevationAngle = 0;
                slider1.Maximum = _sensor.MaxElevationAngle;
                slider1.Minimum = _sensor.MinElevationAngle;
                slider1.Value = _sensor.ElevationAngle;
            }

            skBrushes[0] = Brushes.Yellow;
            skBrushes[1] = Brushes.Blue;
            skBrushes[2] = Brushes.Red;
            skBrushes[3] = Brushes.Green;
            skBrushes[4] = Brushes.HotPink;
            skBrushes[5] = Brushes.DarkOrange;
            skBrushes[6] = Brushes.Brown;

            
            //haveNewFormat = true;
        }

        void sensor_AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            //using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
            //{
            //    if (depthFrame == null)
            //    {
            //        return;
            //    }

            //    byte[] depthPixels = GenerateColoredBytes(depthFrame);

            //    int stride_depth = depthFrame.Width * 4;

            //    if (haveNewFormat)
            //    {
            //        this.outputBitmap = new WriteableBitmap(
            //            depthFrame.Width,
            //            depthFrame.Height,
            //            96,  // DpiX
            //            96,  // DpiY
            //            PixelFormats.Bgr32,
            //            null);

            //        this.image1.Source = this.outputBitmap;
            //        haveNewFormat = false;
            //    }

            //    this.outputBitmap.WritePixels(
            //        new Int32Rect(0, 0, depthFrame.Width, depthFrame.Height),
            //        depthPixels,
            //        depthFrame.Width * Bgr32BytesPerPixel,
            //        0);
            //}

            haveSkeletonData = false;

            using (SkeletonFrame skFrame = e.OpenSkeletonFrame())
            {
                if (skFrame == null)
                {
                    return;
                }

                if (skData == null || skData.Length != skFrame.SkeletonArrayLength)
                {
                    skData = new Skeleton[skFrame.SkeletonArrayLength];
                }
                skFrame.CopySkeletonDataTo(skData);
                haveSkeletonData = true;

            }

            if (haveSkeletonData)
            {
                using (DepthImageFrame depthImageFrame = e.OpenDepthImageFrame())
                {
                    if (depthImageFrame == null)
                    {
                        return;
                    }
                    textBlock1.Text = "Not Tracked";

                    int index = 0;
                    Skeleton otherSK = null;
                    foreach (Skeleton sk in skData)
                    {
                        if (Persons[index] == null)
                        {
                            Persons[index] = new PersonWPF(skBrushes[index]);
                            Persons[index].AddToCanvas(canvas1);
                        }

                        Persons[index].ClearPerson();
                        if (sk.TrackingState != SkeletonTrackingState.Tracked)
                        {
                            index++;
                            continue;
                        }

                        if (otherSK == null) otherSK = sk;

                        jointMapping.Clear();
                        foreach (Joint joint in sk.Joints)
                        {
                            Point mappedPoint = this.GetPosition2DLocation(depthImageFrame, joint.Position);
                            jointMapping[joint.JointType] = mappedPoint;
                            //txtMsg += "\n" + joint.JointType.ToString() + "=(" + mappedPoint.X.ToString() + "," + mappedPoint.Y.ToString() + ")";
                        }

                        textBlock1.Text = sk.TrackingState.ToString();
                        //double pHeight = (jointMapping[JointType.FootLeft].Y + jointMapping[JointType.FootRight].Y) / 2 - jointMapping[JointType.Head].Y;

                        // ### GetRealLength(){ code here:
                        DepthImagePoint depthPoint1 = depthImageFrame.MapFromSkeletonPoint(sk.Joints[JointType.Head].Position);
                        DepthImagePoint depthPoint2 = depthImageFrame.MapFromSkeletonPoint(sk.Joints[JointType.FootLeft].Position);
                        DepthImagePoint dp3 = depthImageFrame.MapFromSkeletonPoint(sk.Position);

                        double depth = (depthPoint1.Depth + depthPoint2.Depth + dp3.Depth) / 3;
                        double lengthPixel = Math.Abs(depthPoint1.Y - depthPoint2.Y);
                        double frameLengthReal = 2 * depth * Math.Tan(43.0 / 2 * Math.PI / 180);

                        // lengthReal/frameLengthReal = lengthPixel/frameLengthPixel
                        double lengthReal = lengthPixel * frameLengthReal / depthImageFrame.Height;
                        // ###       }
                        textBlock1.Text += string.Format("\nDepth:{0}mm\nHReal:{1}mm,FReal:{2}\nHPixel:{3},FPixel:{4}", new object[] { depth.ToString(), ((int)lengthReal).ToString(), frameLengthReal, lengthPixel.ToString(), depthImageFrame.Height.ToString() });
                        
                        if (flag > 0)
                        {
                            double h= GetHeight(depthImageFrame);
                            textBlock1.Text += "\n" + h.ToString() + "," + (h * frameLengthReal / depthImageFrame.Height).ToString();
                        }

                        if (otherSK != null && otherSK != sk)
                        {
                            double distance = Math.Sqrt((Math.Pow((otherSK.Position.X - sk.Position.X), 2) + Math.Pow((otherSK.Position.Z - sk.Position.Z), 2)));
                            textBlock1.Text += "\n Distance:" + distance.ToString();
                        }

                        //double pHeight = GetRealLength(depthImageFrame, sk.Joints[JointType.Head].Position, sk.Joints[JointType.FootLeft].Position);
                        //textBlock1.Text += "\nHeight:" + pHeight.ToString();

                        Persons[index++].PaintPerson(jointMapping);

                        // Look up the center point
                        Point centerPoint = this.GetPosition2DLocation(depthImageFrame, sk.Position);

                        // Scale the skeleton thickness
                        // 1.0 is the desired size at 640 width
                        double scale = this.RenderSize.Width / 640;

                        //textBlock1.Text = txtMsg;
                    }
                }

            }
        }

        /// <summary>
        /// 获取身高
        /// </summary>
        /// <param name="depthFrame">深度图像数据</param>
        /// <returns></returns>
        public int GetHeight(DepthImageFrame depthFrame)
        {
            int height;
            short[] rawDepthData = new short[depthFrame.PixelDataLength];
            depthFrame.CopyPixelDataTo(rawDepthData);

            int Max = 0;
            int Min = 480;
            for (int depthIndex = 0; depthIndex < rawDepthData.Length; depthIndex++)
            {
                int player = rawDepthData[depthIndex] & DepthImageFrame.PlayerIndexBitmask;
                if (player > 0)
                {
                    int TempYdata = depthIndex / 640;
                    if (TempYdata > Max)
                    {
                        Max = TempYdata;
                    }
                    if (TempYdata < Min)
                    {
                        Min = TempYdata;
                    }
                }
            }

            height = Max - Min;
            return height;
        }
        private byte[] GenerateColoredBytes(DepthImageFrame depthFrame)
        {
            //get the raw data from kinect with the depth for every pixel
            short[] rawDepthData = new short[depthFrame.PixelDataLength];
            depthFrame.CopyPixelDataTo(rawDepthData);

            //byte[] rawBytesDepthData = File.ReadAllBytes(@"E:\kin_rawDepth.Data");
            //for (int i = 0; i < rawDepthData.Length; i++)
            //{
            //    byte[] tmp = System.BitConverter.GetBytes(rawDepthData[i]);
            //    rawBytesDepthData[i * 2] = tmp[0];
            //    rawBytesDepthData[i * 2 + 1] = tmp[0];
            //}


            //use depthFrame to create the image to display on-screen
            //depthFrame contains color information for all pixels in image
            //Height * Width * 4(Red, Green, Blue, empth byte)
            Byte[] pixels = new byte[depthFrame.Height * depthFrame.Width * 4];

            //Bgr32  - Blue, Green, Red, empth byte
            //Bgra32 - Blue, Green, Red, transparency
            //You must set transparency for Bgra as .NET defaults a byte to 0 = fully transparency

            //hardcoded locations to Blue, Green, Red(RGB) index positions

            const int BlueIndex = 0;
            const int GreenIndex = 1;
            const int RedIndex = 2;

            //loop through all distances
            //pick a RGB color based on distance
            for (int depthIndex = 0, colorIndex = 0;
                depthIndex < rawDepthData.Length && colorIndex < pixels.Length;
                depthIndex++, colorIndex += 4)
            {
                //get the player(requires skeleton tracking enabled for values)
                int player = rawDepthData[depthIndex] & DepthImageFrame.PlayerIndexBitmask;

                //gets the depth value
                int depth = rawDepthData[depthIndex] >> DepthImageFrame.PlayerIndexBitmaskWidth;

                //.9M or 2.95'
                if (depth <= 900)
                {
                    //we are very close
                    pixels[colorIndex + BlueIndex] = 255;
                    pixels[colorIndex + GreenIndex] = 0;
                    pixels[colorIndex + RedIndex] = 0;
                }
                // 0.9M - 2, or 2.95'   -  6.56'
                else if (depth > 900 && depth < 2000)
                {
                    //we are a bit further away
                    pixels[colorIndex + BlueIndex] = 0;
                    pixels[colorIndex + GreenIndex] = 255;
                    pixels[colorIndex + RedIndex] = 0;
                }
                else if (depth > 2000)
                {
                    //we are the farthest
                    pixels[colorIndex + BlueIndex] = 0;
                    pixels[colorIndex + GreenIndex] = 0;
                    pixels[colorIndex + RedIndex] = 255;
                }

                //equal coloring for monochromatic histogram
                byte intensity = calculateIntensityFromDepth(depth);
                pixels[colorIndex + BlueIndex] = intensity;
                pixels[colorIndex + GreenIndex] = intensity;
                pixels[colorIndex + RedIndex] = intensity;

                //Color all players "gold"
                if (player > 0)
                {
                    pixels[colorIndex + BlueIndex] = Colors.Gold.B;
                    pixels[colorIndex + GreenIndex] = Colors.Gold.G;
                    pixels[colorIndex + RedIndex] = Colors.Gold.R;
                }
            }

            return pixels;
        }

        private byte calculateIntensityFromDepth(int depth)
        {
            //formula for calculating monochrome intensity for histogram
            return (byte)(255 - (255 * Math.Max(depth - MinDepthDistance, 0)
                / (MaxDepthDistanceOffset)));
        }

        private Point GetPosition2DLocation(DepthImageFrame depthFrame, SkeletonPoint skeletonPoint)
        {
            DepthImagePoint depthPoint = depthFrame.MapFromSkeletonPoint(skeletonPoint);

            ColorImagePoint colorPoint = depthFrame.MapToColorImagePoint(depthPoint.X, depthPoint.Y, this._sensor.ColorStream.Format);

            // map back to skeleton.Width & skeleton.Height
            //return new Point(
            //    (int)(this.RenderSize.Width * colorPoint.X / this._sensor.ColorStream.FrameWidth),
            //    (int)(this.RenderSize.Height * colorPoint.Y / this._sensor.ColorStream.FrameHeight));
            return new Point(
                (int)(canvas1.Width * colorPoint.X / this._sensor.ColorStream.FrameWidth),
                (int)(canvas1.Height * colorPoint.Y / this._sensor.ColorStream.FrameHeight));

        }

        private double GetRealLength(DepthImageFrame depthFrame, SkeletonPoint skeletonPoint1,SkeletonPoint skeletonPoint2)
        {
            DepthImagePoint depthPoint1 = depthFrame.MapFromSkeletonPoint(skeletonPoint1);
            DepthImagePoint depthPoint2 = depthFrame.MapFromSkeletonPoint(skeletonPoint2);

            double depth = (depthPoint1.Depth+depthPoint2.Depth)/2;
            //double lengthPixel = Math.Sqrt((depthPoint1.X-depthPoint2.X)^2+(depthPoint1.Y-depthPoint2.Y)^2);
            
            double lengthPixel = Math.Abs(depthPoint1.Y - depthPoint2.Y);
            double frameLengthReal = 2 * depth * Math.Tan(57 / 2 * Math.PI / 180);
            
            // lengthReal/frameLengthReal = lengthPixel/frameLengthPixel
            double lengthReal = lengthPixel * frameLengthReal / depthFrame.Height;
            return lengthReal;
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            StopKinect(kinectSensorChooser1.Kinect);
        }

        void StopKinect(KinectSensor sensor)
        {
            if (sensor != null)
            {
                sensor.Stop();
                sensor.AudioSource.Stop();
            }
        }

        void kinectSensorChooser_KinectSensorChanged(object sender, DependencyPropertyChangedEventArgs e)
        {
            KinectSensor oldSensor = (KinectSensor)e.OldValue;
            StopKinect(oldSensor);

            KinectSensor newSensor = (KinectSensor)e.NewValue;
            newSensor.ColorStream.Enable();
            newSensor.DepthStream.Enable();
            newSensor.SkeletonStream.Enable();
            newSensor.AllFramesReady += new EventHandler<AllFramesReadyEventArgs>(sensor_AllFramesReady);
            kinectColorViewer1.Kinect = newSensor;
            try
            {
                newSensor.Start();
            }
            catch (System.IO.IOException)
            {
                kinectSensorChooser1.AppConflictOccurred();
            }
            _sensor = newSensor;
            slider1.Maximum = _sensor.MaxElevationAngle;
            slider1.Minimum = _sensor.MinElevationAngle;
            slider1.Value = _sensor.ElevationAngle;

        }

        private void btnMove_Click(object sender, RoutedEventArgs e)
        {
            if (_sensor != null)
            {
                _sensor.ElevationAngle = (int)slider1.Value;
            }
        }

        private void textBlock2_MouseDown(object sender, MouseButtonEventArgs e)
        {
            textBlock2.Text = textBlock1.Text;
            //flag = -flag;
        }
    }

    public class PersonWPF
    {
        public Ellipse Head;
        public Polyline BodyMain = new Polyline();
        public Polyline BodyHands = new Polyline();
        public Polyline BodyFoots = new Polyline();
        private PointCollection pc1 = new PointCollection();
        private PointCollection pc2 = new PointCollection();
        private PointCollection pc3 = new PointCollection();
        private Brush pBrush;
        private bool cleared;

        public PersonWPF(Brush brush)
        {
            pBrush = brush;
            Head = new Ellipse();
            Head.Height = 20;
            Head.Width = 20;
            BodyMain.Stroke = BodyHands.Stroke = BodyFoots.Stroke = pBrush;
            BodyMain.Points = pc1;
            BodyHands.Points = pc2;
            BodyFoots.Points = pc3;
            cleared = true;
        }

        public void AddToCanvas(Canvas can)
        {
            can.Children.Add(Head);
            can.Children.Add(BodyMain);
            can.Children.Add(BodyHands);
            can.Children.Add(BodyFoots);
        }

        public void ClearPerson()
        {
            if (!cleared)
            {
                Head.Stroke = Brushes.Transparent;
                pc1.Clear();
                pc2.Clear();
                pc3.Clear();
                cleared = true;
            }
        }

        public void PaintPerson(Dictionary<JointType, Point> mapedJoints)
        {
            double headX = mapedJoints[JointType.Head].X - 10;
            double headY = mapedJoints[JointType.Head].Y - 10;
            Head.Margin = new Thickness(headX, headY, 0, 0);
            Head.Stroke = pBrush;

            pc1.Add(mapedJoints[JointType.Head]);
            pc1.Add(mapedJoints[JointType.ShoulderCenter]);
            pc1.Add(mapedJoints[JointType.Spine]);
            pc1.Add(mapedJoints[JointType.HipCenter]);

            pc2.Add(mapedJoints[JointType.HandRight]);
            pc2.Add(mapedJoints[JointType.WristRight]);
            pc2.Add(mapedJoints[JointType.ElbowRight]);
            pc2.Add(mapedJoints[JointType.ShoulderRight]);
            pc2.Add(mapedJoints[JointType.ShoulderCenter]);
            pc2.Add(mapedJoints[JointType.ShoulderLeft]);
            pc2.Add(mapedJoints[JointType.ElbowLeft]);
            pc2.Add(mapedJoints[JointType.WristLeft]);
            pc2.Add(mapedJoints[JointType.HandLeft]);

            pc3.Add(mapedJoints[JointType.FootRight]);
            pc3.Add(mapedJoints[JointType.AnkleRight]);
            pc3.Add(mapedJoints[JointType.KneeRight]);
            pc3.Add(mapedJoints[JointType.HipRight]);
            pc3.Add(mapedJoints[JointType.HipCenter]);
            pc3.Add(mapedJoints[JointType.HipLeft]);
            pc3.Add(mapedJoints[JointType.KneeLeft]);
            pc3.Add(mapedJoints[JointType.AnkleLeft]);
            pc3.Add(mapedJoints[JointType.FootLeft]);

            cleared = false;
        }
    }
}
