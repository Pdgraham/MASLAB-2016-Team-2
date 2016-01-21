#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <list>
#include <string>
#include <map>
#include <vector>
#include <unordered_set>
#include <set>
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


list<list<int> > FloodFill(int orig_x, int orig_y, cv::Mat image) {
    list< list<int> > queue;
    set<list< int > > processed;
    list< list<int> > output;
    
    list<int> coor;
    list<int> output_coor;
    coor.push_back(orig_x);
    coor.push_back(orig_y);
    queue.push_back(coor);

    uint8_t* pixelPtr = (uint8_t*)image.data;
    int cn = image.channels();
    int width = image.cols;
    int height = image.rows;
    Scalar_<uint8_t> bgrPixel;

    // cout << "orig_x: " << orig_x << "  orig_y : " << orig_y << endl;

    while (queue.size() != 0) {
        // cout << "Inside queue not empty.";
        list<int> cur_coor = queue.front();
        queue.pop_front();
        int x = cur_coor.front();
        cur_coor.pop_front();
        int y = cur_coor.front();
        cur_coor.pop_front();
        cur_coor.push_back(x);
        cur_coor.push_back(y);
        bgrPixel.val[0] = pixelPtr[x*width*cn + y*cn + 0]; // B
        bgrPixel.val[1] = pixelPtr[x*width*cn + y*cn + 1]; // G
        bgrPixel.val[2] = pixelPtr[x*width*cn + y*cn + 2]; // R
        // Vec3b bgrPixel = image.at<Vec3b>(x, y);
        // bgrPixel.at(0);

        // cout << "R: ";
        // cout << bgrPixel.val[2];
        // cout << "  G: ";
        // cout << bgrPixel.val[1];
        // cout << "  B: ";
        // cout << bgrPixel.val[0];
        // cout << endl;

        // Check if current pixel is red
        // cout << "x: "<< x << "  y: " << y << endl;
        // cout << "R: ";
        // cout << int(bgrPixel.val[2]);
        // cout << endl;
        if (bgrPixel.val[2] > 1.3*bgrPixel.val[1] && bgrPixel.val[2] > 1.3*bgrPixel.val[0]) {
            output.push_back(cur_coor);
            // cout << "Pixel is red";

            for (int dx = -1; dx < 2; dx+=2) {
                for (int dy = -1; dy < 2; dy+=2) {
                    if (x+dx < width && x+dx > 0 && y+dy < height && y+dy > 0) {
                        // cout << "x + dx: ";
                        // cout << x + dx;
                        // cout << endl;
                        list<int> neighbor_coor;
                        neighbor_coor.push_back(x + dx);
                        neighbor_coor.push_back(y + dy);
                        set<list<int> >::const_iterator got = processed.find(neighbor_coor);

                        if (got == processed.end()) {
                            // not in processed
                            processed.insert(neighbor_coor);
                            // cout << "Inside not processed";
                            queue.push_back(neighbor_coor);
                        }
                    }
                }
            }
        }

        // processed.insert(coor);
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

    // for(int i = 0; i < image.rows; i++) {
    //     for(int j = 0; j < image.cols; j++) {
    //         bgrPixel.val[0] = pixelPtr[i*image.cols*cn + j*cn + 0]; // B
    //         bgrPixel.val[1] = pixelPtr[i*image.cols*cn + j*cn + 1]; // G
    //         bgrPixel.val[2] = pixelPtr[i*image.cols*cn + j*cn + 2]; // R

    //         if (bgrPixel.val[2] > 1.3*bgrPixel.val[1] && bgrPixel.val[2] > 1.3*bgrPixel.val[0]) {
    //             pixelPtr[i*image.cols*cn + j*cn + 0] = 0;
    //             pixelPtr[i*image.cols*cn + j*cn + 1] = 0;
    //             pixelPtr[i*image.cols*cn + j*cn + 2] = 0;  
    //         }
    //     }
    // }

    // cout << image << endl;

    list<list<int> > output;
    cout << "FloodFill output: " << endl;
    // std::string color = "Red";
    for (int x = 0; x < height; x+=50) { // why does switching height and width work?
        for (int y = 0; y < width; y+=50) {
            output = FloodFill(x, y, image);
            for (list<int> outerList : output) {
                // cout << "Inside!";
                for (int pixelVal : outerList) {
                    cout << int(pixelVal) << " ";
                }
            } 
            cout << endl;
            cout << endl;
        }
    }

    // for (std::list< list<int> >::iterator output_it = output.begin(); output_it != output.end(); output_it++) {
    //     // for (output_it = output.begin(); output_it != output.end(); output_it++) {
    //     for (std::list< int >::iterator output2_it = output_it.begin(); output2_it != output_it.end(); output2_it++) {
    //         cout << " " << *output2_it;
    //     }
    // }
    cout << endl;

    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    imshow( "Display window", image );                   // Show our image inside it.

    waitKey(0);                                          // Wait for a keystroke in the window
    return 0;
}