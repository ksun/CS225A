using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Research.Kinect.Nui;

namespace KinectDepthSmoothing
{
    public partial class MainWindow : Window
    {
        private BitmapSource CreateImageFromDepthImage(ImageFrame image)
        {
            int width = image.Image.Width;
            int height = image.Image.Height;

            var depthFrame = image.Image.Bits;
            // We multiply the product of width and height by 4 because each byte
            // will represent a different color channel per pixel in the final iamge.
            var colorFrame = new byte[height * width * 4];

            // Process each row in parallel
            Parallel.For(0,240, depthRowIndex =>
            {
                //  Within each row, we then iterate over each 2 indexs to be combined into a single depth value
                for (int depthColumnIndex = 0; depthColumnIndex < 640; depthColumnIndex += 2)
                {
                    var depthIndex = depthColumnIndex + (depthRowIndex * 640);

                    // Because the colorFrame we are creating has twice as many bytes representing
                    // a pixel in the final image, we set the index to be twice of the depth index.
                    var index = depthIndex * 2;

                    // Calculate the distance represented by the two depth bytes
                    var distance = CalculateDistanceFromDepth(depthFrame[depthIndex], depthFrame[depthIndex + 1]);

                    // Map the distance to an intesity that can be represented in RGB
                    var intensity = CalculateIntensityFromDistance(distance);

                    // Apply the intensity to the color channels
                    colorFrame[index + BlueIndex] = intensity;
                    colorFrame[index + GreenIndex] = intensity;
                    colorFrame[index + RedIndex] = intensity;
                }
            });

            return BitmapSource.Create(width, height, 96, 96, PixelFormats.Bgr32, null, colorFrame, width * PixelFormats.Bgr32.BitsPerPixel / 8);
        }
    }
}
