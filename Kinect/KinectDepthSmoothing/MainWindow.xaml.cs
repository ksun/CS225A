using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Controls;
using Microsoft.Research.Kinect.Nui;

namespace KinectDepthSmoothing
{
    /// <summary>
    /// This program was created by Karl Sanford for submission to The Code Project in conjunction with an article
    /// explaining the techniques used within.  This program will smooth Depth Images from the Kinect using two techniques:
    /// Pixel Filtering and Weighted Moving Average.  It will also allow users of the program to change parameters of these
    /// techniques in the UI to see their effect on the resulting images.
    /// 
    /// I have documented the code where I thought it might help people, but if you have any questions/comments/concerns about
    /// this program, please feel free to contact me in the comments section for this article on The Code Project.
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
        }

        #region Private Fields
        // The Kinect runtime used to recieve depth data
        private Runtime runtime;

        // Fields used to adjust the filtering feature
        // Initial values are specified in XAML
        private bool useFiltering;
        // Will specify how many non-zero pixels within a 1 pixel band
        // around the origin there should be before a filter is applied
        private int innerBandThreshold;
        // Will specify how many non-zero pixels within a 2 pixel band
        // around the origin there should be before a filter is applied
        private int outerBandThreshold;

        // Fields used to adjust the averaging feature
        // Initial values are specified in XAML
        private bool useAverage;
        // Will specify how many frames to hold in the Queue for averaging
        private int averageFrameCount;
        // The actual Queue that will hold all of the frames to be averaged
        private Queue<short[]> averageQueue = new Queue<short[]>();

        // Fields used for FPS calculations
        private int totalFrames;
        private int lastFrames;
        private DateTime lastTime;
        
        // Constants used to address the individual color pixels for generating images
        private const int RedIndex = 2;
        private const int GreenIndex = 1;
        private const int BlueIndex = 0;

        // Constants used to map value ranges for distance to pixel intensity conversions
        private const int MaxDepthDistance = 4000;
        private const int MinDepthDistance = 850;
        private const int MaxDepthDistanceOffset = 3150;
        #endregion Private Fields

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            // If you don't make sure the Kinect is plugged in and working before trying to use it, the app will crash
            if (Runtime.Kinects.Count > 0)
            {
                runtime = Runtime.Kinects[0];

                runtime.Initialize(RuntimeOptions.UseDepth);

                runtime.DepthFrameReady += runtime_DepthFrameReady;

                runtime.DepthStream.Open(ImageStreamType.Depth, 2, ImageResolution.Resolution320x240, ImageType.Depth);
            }
            else
                MessageBox.Show("Sorry, your Kinect could not be found.");
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            // If you don't uninitialize on closing, sometimes the Kinect is not available
            // the next time you run the program and you have to re-attach the Kinect.
            if (runtime != null)
                runtime.Uninitialize();
        }

        private void runtime_DepthFrameReady(object sender, ImageFrameReadyEventArgs e)
        {
            // The Raw Image is simply the raw depth data from the Kinect displayed as an image
            // This is the method I had been using for a while, but was unsatisfied with, which
            // lead me to create the next method.
            ImageRaw.Source = CreateImageFromDepthImage(e.ImageFrame);

            // The Smoothed Image will apply both a filter and a weighted moving average over the
            // depth data from the Kinect (depending on UI selections)
            ImageSmooth.Source = CreateSmoothImageFromDepthArray(e.ImageFrame);

            // Update the FPS after every frame so we can monitor our performance and make sure
            // that the cost of aesthetics doesn't override performance
            UpdateFps();
        }

        private void UpdateFps()
        {
            totalFrames++;
            DateTime current = DateTime.Now;
            if (lastTime == DateTime.MaxValue || current.Subtract(lastTime) > TimeSpan.FromSeconds(1))
            {
                TextblockFps.Text = (totalFrames - lastFrames).ToString();
                lastFrames = totalFrames;
                lastTime = current;
            }
        }

        private short CalculateDistanceFromDepth(byte first, byte second)
        {
            // Please note that this would be different if you use Depth and User tracking rather than just depth
            return (short)(first | second << 8);
        }

        private byte CalculateIntensityFromDistance(int distance)
        {
            // This will map a distance value to a 0 - 255 range
            // for the purposes of applying the resulting value
            // to RGB pixels.
            int newMax = distance - MinDepthDistance;
            if (newMax > 0)
                return (byte)(255 - (255 * newMax
                / (MaxDepthDistanceOffset)));
            else
                return (byte)255;
        }

        #region UI Event Handlers
        private void CheckboxUseFiltering_Checked(object sender, RoutedEventArgs e)
        {
            useFiltering = ((CheckBox)sender).IsChecked.Value;
        }

        private void SliderInnerBand_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            innerBandThreshold = (int)e.NewValue;
        }

        private void SliderOuterBand_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            outerBandThreshold = (int)e.NewValue;
        }

        private void CheckboxUseAverage_Checked(object sender, RoutedEventArgs e)
        {
            useAverage = ((CheckBox)sender).IsChecked.Value;

            // If you don't clear out the Queue when you turn off this feature, then you turn it back on
            // you will get a few frames of very odd results if you are moving around in the frames.
            if (!useAverage)
                averageQueue.Clear();
        }

        private void SliderAverage_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            averageFrameCount = (int)e.NewValue;
        }
        #endregion UI Event Handlers
    }
}
