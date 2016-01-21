#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <list>
#include <string>
#include <map>
#include <vector>
#include <unordered_set>
#include <set>
#include <time.h>

using namespace cv;
using namespace std;

namespace
{
    template<typename T>
    std::size_t make_hash(const T& v)
    {
        return std::hash<T>()(v);
    }

    void hash_combine(std::size_t& h, const std::size_t& v)
    {
        h ^= v + 0x9e3779b9 + (h << 6) + (h >> 2);
    }

    template<typename T>
    struct hash_container
    {
        size_t operator()(const T& v) const
        {
            size_t h=0;
            for( const auto& e : v ) {
                hash_combine(h, make_hash(e));
            }
            return h;
        }
    };
}

namespace std
{
    template<typename T, typename U>
    struct hash< pair<T, U> >
    {
        size_t operator()(const pair<T,U>& v) const
        {
            size_t h=make_hash(v.first);
            hash_combine(h, make_hash(v.second));
            return h;
        }
    };

    template<typename... T>
    struct hash< vector<T...> > : hash_container< vector<T...> > {};

    template<typename... T>
    struct hash< map<T...> > : hash_container< map<T...> > {};
}


// Flood-fill (node, target-color, replacement-color):
//  1. If target-color is equal to replacement-color, return.
//  2. Set Q to the empty queue.
//  3. Add node to the end of Q.
//  4. While Q is not empty:
//  5.     Set n equal to the first element of Q.
//  6.     Remove first element from Q.
//  7.     If the color of n is equal to target-color:
//  8.         Set the color of n to replacement-color and mark "n" as processed.
//  9.         Add west node to end of Q if west has not been processed yet.
//  10.        Add east node to end of Q if east has not been processed yet.
//  11.        Add north node to end of Q if north has not been processed yet.
//  12.        Add south node to end of Q if south has not been processed yet.
//  13. Return.

void FloodFill(int orig_x, int orig_y, cv::Mat image) {
    int topLeftCornerY = 1200; // must be greater than 640x480=1120
    int topLeftCornerX = 1200; // must be greater than 640x480=1120
    int bottomRightCornerY = 0;
    int bottomRightCornerX = 0;

    uint8_t* pixelPtr = (uint8_t*)image.data;
    int cn = image.channels();
    int width = image.cols;
    int height = image.rows;
    Scalar_<uint8_t> bgrPixel;

    // cout << "orig_x: " << orig_x << "  orig_y : " << orig_y << endl;

    bgrPixel.val[0] = pixelPtr[orig_x*width*cn + orig_y*cn + 0]; // B
    bgrPixel.val[1] = pixelPtr[orig_x*width*cn + orig_y*cn + 1]; // G
    bgrPixel.val[2] = pixelPtr[orig_x*width*cn + orig_y*cn + 2]; // R

    if (bgrPixel.val[2] > 1.3*bgrPixel.val[1] && bgrPixel.val[2] > 1.3*bgrPixel.val[0]) {

//        pixelPtr[orig_x*width*cn + orig_y*cn + 0] = 0;
//        pixelPtr[orig_x*width*cn + orig_y*cn + 1] = 0;
        pixelPtr[orig_x*width*cn + orig_y*cn + 2] = 0;

        for (int dx = -1; dx < 2; dx++) {
            for (int dy = -1; dy < 2; dy++) {
                if (dx == 0 && dy == 0) {
                    continue;
                }
                if (orig_x+dx < width && orig_x+dx >= 0 && 
                    orig_y+dy < height && orig_y+dy >= 0 &&
//                    pixelPtr[(orig_x+dx)*width*cn + (orig_y+dy)*cn + 0] != 0 &&
//                    pixelPtr[(orig_x+dx)*width*cn + (orig_y+dy)*cn + 1] != 0 &&
                    pixelPtr[(orig_x+dx)*width*cn + (orig_y+dy)*cn + 2] != 0) {

                    FloodFill(orig_x+dx, orig_y+dy, image);

                }
            }
        }
    }

    return;
}


int main( int argc, char** argv )
{
    const clock_t begin_time = clock();
    Mat image = imread("block_test.jpg");
    int height = image.rows;
    int width = image.cols;

    uint8_t* pixelPtr = (uint8_t*)image.data;
    int cn = image.channels();
    Scalar_<uint8_t> bgrPixel;

    list<int> output;
    int numBlocks = 0;
    cout << "FloodFill output: " << endl;
    for (int x = 0; x < height; x+=25) { // why does switching height and width work?
        for (int y = 0; y < width; y+=25) {
            // output = FloodFill(480, 1990, image);
            FloodFill(x, y, image);

            cout << endl;
            if (output.size() !=0) {
                numBlocks++;
            }
        }
    }
    std::cout << "Flood fill took " << float( clock () - begin_time ) /  CLOCKS_PER_SEC << "s." << endl;
    cout << "Number of Flood Fill Areas discovered: " << numBlocks << endl;

    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", image );                   // Show our image inside it.

    waitKey(0);                                          // Wait for a keystroke in the window
    return 0;
}

