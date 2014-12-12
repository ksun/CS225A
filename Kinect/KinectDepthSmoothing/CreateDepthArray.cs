using System.Threading.Tasks;
using System.Windows;
using Microsoft.Research.Kinect.Nui;

namespace KinectDepthSmoothing
{
    public partial class MainWindow : Window
    {
        private short[] CreateDepthArray(ImageFrame image)
        {
            // When creating a depth array, it will have half the number of indexes than the original depth image
            // This is because the depth image uses two bytes to represent depth.  These values must then be 
            // transformed to a single value per pixel of the final image that represents depth
            // for purposes of smoothing prior to rendering.

            short[] returnArray = new short[image.Image.Width * image.Image.Height];
            byte[] depthFrame = image.Image.Bits;

            Parallel.For(0, 240, depthImageRowIndex =>
            {
                for (int depthImageColumnIndex = 0; depthImageColumnIndex < 640; depthImageColumnIndex += 2)
                {
                    var depthIndex = depthImageColumnIndex + (depthImageRowIndex * 640);
                    var index = depthIndex / 2;

                    returnArray[index] = CalculateDistanceFromDepth(depthFrame[depthIndex], depthFrame[depthIndex + 1]);
                }
            });

            return returnArray;
        }
    }
}
