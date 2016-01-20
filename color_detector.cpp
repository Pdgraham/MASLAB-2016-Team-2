#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <list>
#include <vector>
#include <unordered_set>
// #include "flood_fill.h"

using namespace cv;
using namespace std;

list<vector<int> > FloodFill(int x, int y, cv::Mat image) {
    list< vector<int> > queue;
    unordered_set<list<int> > processed;
    list< vector<int> > output;
    
    vector<int> coor;
    int neighbor_coor[2];
    list<int> output_coor;
    coor.at(0) = x;
    coor.at(1) = y;
    queue.emplace_back(coor);

    uint8_t* pixelPtr = (uint8_t*)image.data;
    int cn = image.channels();
    int width = image.cols;
    int height = image.rows;
    Scalar_<uint8_t> bgrPixel;

    while (queue.size() != 0) {
        vector<int> cur_coor = queue.front();
        queue.pop_front();
        int x = cur_coor.at(0);
        int y = cur_coor.at(1);
        bgrPixel.val[0] = pixelPtr[x*width*cn + y*cn + 0]; // B
        bgrPixel.val[1] = pixelPtr[x*width*cn + y*cn + 1]; // G
        bgrPixel.val[2] = pixelPtr[x*width*cn + y*cn + 2]; // R


        // Check if current pixel is red
        if (bgrPixel.val[2] > 1.3*bgrPixel.val[1] && bgrPixel.val[2] > 1.3*bgrPixel.val[0]) {
            output.emplace_back(coor);

            for (int dx = -1; dx < 2; dx+=2) {
                for (int dy = -1; dy < 2; dy+=2) {
                    if (x < width && x > 0 && y < height && y > 0) {
                        neighbor_coor[0] = x + dx;
                        neighbor_coor[1] = y + dy;
                        unordered_set<list<int> >::const_iterator got = processed.find(neighbor_coor);

                        if (got == processed.end())
                            // not in processed
                            queue.emplace_back(neighbor_coor);
                    }
                }
              }
        }

    processed.push(processed);
    }

    return output;
}

int main( int argc, char** argv )
{
    // if( argc != 2)
    // {
    //  cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
    //  return -1;
    // }

    Mat image;
    image = imread("block_test.jpg");   // Read the file
    int height = image.rows;
    int width = image.cols;

    uint8_t* pixelPtr = (uint8_t*)image.data;
    int cn = image.channels();
    Scalar_<uint8_t> bgrPixel;

    for(int i = 0; i < image.rows; i++) {
        for(int j = 0; j < image.cols; j++) {
            bgrPixel.val[0] = pixelPtr[i*image.cols*cn + j*cn + 0]; // B
            bgrPixel.val[1] = pixelPtr[i*image.cols*cn + j*cn + 1]; // G
            bgrPixel.val[2] = pixelPtr[i*image.cols*cn + j*cn + 2]; // R

            if (bgrPixel.val[2] > 1.3*bgrPixel.val[1] && bgrPixel.val[2] > 1.3*bgrPixel.val[0]) {
                pixelPtr[i*image.cols*cn + j*cn + 0] = 0;
                pixelPtr[i*image.cols*cn + j*cn + 1] = 0;
                pixelPtr[i*image.cols*cn + j*cn + 2] = 0;  
            }
        }
    }

    // std::string color = "Red";
    FloodFill(100, 100, image);

    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", image );                   // Show our image inside it.

    waitKey(0);                                          // Wait for a keystroke in the window
    return 0;
}