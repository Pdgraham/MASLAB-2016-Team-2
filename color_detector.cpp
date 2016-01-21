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
// #include "flood_fill.h"

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

vector<int> FloodFill(int orig_x, int orig_y, cv::Mat image) {
    vector< vector<int> > queue;
    // set< int > processed;
    int topLeftCornerY = 1200; // must be greater than 640x480=1120
    int topLeftCornerX = 1200; // must be greater than 640x480=1120
    int bottomRightCornerY = 0;
    int bottomRightCornerX = 0;

    vector<int> coor;
    coor.push_back(orig_x);
    coor.push_back(orig_y);
    queue.push_back(coor);

    uint8_t* pixelPtr = (uint8_t*)image.data;
    int cn = image.channels();
    int width = image.cols;
    int height = image.rows;
    Scalar_<uint8_t> bgrPixel;  

    while (queue.size() != 0) {
        vector<int> cur_coor = queue.back();
        queue.pop_back();
        int x = cur_coor.at(0);
        int y = cur_coor.at(1);
        bgrPixel.val[0] = pixelPtr[x*width*cn + y*cn + 0]; // B
        bgrPixel.val[1] = pixelPtr[x*width*cn + y*cn + 1]; // G
        bgrPixel.val[2] = pixelPtr[x*width*cn + y*cn + 2]; // R

        bool isRed = bgrPixel.val[2] > 1.3*bgrPixel.val[1] && bgrPixel.val[2] > 1.3*bgrPixel.val[0];
        if (isRed) {
            if (x + y < topLeftCornerX + topLeftCornerY) {
                topLeftCornerX = x;
                topLeftCornerY = y;
            }
            else if (x + y > bottomRightCornerX + bottomRightCornerY) {
                bottomRightCornerX = x;
                bottomRightCornerY = y;
            }

            pixelPtr[x*width*cn + y*cn + 0] = 0;
            pixelPtr[x*width*cn + y*cn + 1] = 0;
            pixelPtr[x*width*cn + y*cn + 2] = 0;

            for (int dx = -1; dx < 2; dx++) {
                for (int dy = -1; dy < 2; dy++) {
                    if (dx == 0 && dy == 0) {
                        continue;
                    }
                    if (x+dx < width && x+dx >= 0 && y+dy < height && y+dy >= 0) {
                        vector<int> neighbor_coor;
                        neighbor_coor.push_back(x + dx);
                        neighbor_coor.push_back(y + dy);
                        // set<list<int> >::const_iterator got = processed.find(neighbor_coor);

                        // if (got == processed.end()) {
                            // not in processed
                            // processed.insert(neighbor_coor);
                            queue.push_back(neighbor_coor);
                        // }
                    }
                }
            }
        }
    }

    vector<int> output;
    int minNumLength = 25;
    if (bottomRightCornerX - topLeftCornerX > minNumLength && bottomRightCornerY - topLeftCornerY > minNumLength) {
        output.push_back(topLeftCornerX);
        output.push_back(topLeftCornerY);
        output.push_back(bottomRightCornerX);
        output.push_back(bottomRightCornerY);
    }

    return output;   
}

int main( int argc, char** argv )
{
    const clock_t begin_time = clock();
    Mat image = imread("block_test.jpg");
    int height = image.rows;
    int width = image.cols;

    vector<int> output;
    int numBlocks = 0;
    for (int x = 0; x < height; x+=50) { // why does switching height and width work?
        for (int y = 0; y < width; y+=50) {
            // output = FloodFill(480, 1990, image);
            output = FloodFill(x, y, image);
            // output = FloodFill(482, 0, image);
            // bgrPixel.val[0] = pixelPtr[x*width*cn + y*cn + 0]; // B
            // bgrPixel.val[1] = pixelPtr[x*width*cn + y*cn + 1]; // G
            // bgrPixel.val[2] = pixelPtr[x*width*cn + y*cn + 2]; // R

            // if (bgrPixel.val[2] > 1.3*bgrPixel.val[1] && bgrPixel.val[2] > 1.3*bgrPixel.val[0]) {
                // output = FloodFill(x, y, image);
                // for (list<int> outerList : output) {
                    // cout << "Inside!";
                    // for (int pixelVal : outerList) {
                    // for (int pixelVal : output) {
                        // cout << int(pixelVal) << " ";
                        // int i = outerList.front();
                        // outerList.pop_front();
                        // int j = outerList.front();
                        // Color the red pixels a different pixel so Flood fill doesn't pick it up again.
                        // pixelPtr[i*image.cols*cn + j*cn + 0] = fillColor[fillIndex];
                        // pixelPtr[i*image.cols*cn + j*cn + 1] = fillColor[fillIndex];
                        // pixelPtr[i*image.cols*cn + j*cn + 2] = fillColor[fillIndex];  
                    // }
                // } 
                // cout << endl;
                if (output.size() !=0) {
                    cout << "[(" << output.at(0) << "," << output.at(1) << ")  (" << output.at(2) << "," << output.at(3) << ")]    ";
                    numBlocks++;
                }
        }
    }
    cout << endl << "Flood fill took " << float( clock () - begin_time ) /  CLOCKS_PER_SEC << "s." << endl;
    cout << "Number of Flood Fill Areas discovered: " << numBlocks << endl;

    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", image );                   // Show our image inside it.

    waitKey(0);                                          // Wait for a keystroke in the window
    return 0;
}