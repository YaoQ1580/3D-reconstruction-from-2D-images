# 3D-reconstruction-from-2D-images

**1.	Introduction**

This software can reconstruct a 3D surface from several pictures taken from different perspectives and from different cameras, as depicted in Fig1. This technology may be applied to 3D print industry.
 ![image](https://user-images.githubusercontent.com/67689632/200151754-5f3ad833-32bc-4526-8e96-14b1ffd5dfc6.png)

Fig 1

**2.	Implementation**

The software can be split into two parts:

1. retrieve 3D point coordinate from 2D pictures

2. generate surface topology from these 3D points

**2.1 retrieve 3D point coordinate from 2D pictures**

Actually, every pixel of the 2D images are projections from 3D points, as depict in Fig2. To get the coordinate value of 3D points, firstly, we need to find which pair of pixels from different pictures are actually projections from the same 3D point.
 ![image](https://user-images.githubusercontent.com/67689632/200151759-3c0d83a8-44f9-4432-9aed-413b5f78dbb6.png)

Fig 2
This is not an easy job because usually the line connecting two matched pixels are not horizontal, as depicted in Fig3, which means that to find a pixel’s matched pixel, you need to search different rows instead of the same row of the given pixel. This will add some computation complexity.
 ![image](https://user-images.githubusercontent.com/67689632/200151760-2f5237eb-36ba-4758-9f44-87acc1077859.png)

Fig 3

**2.1.1 3D rectification**

To facilitate the process of matching pixel, usually a 3D rectification procedure is performed so that all matched pixels are in the same row. To do this 3D rectification, several parameters are needed, which can be easily acquired from some mature algorithm libraries.

**2.1.2 match pixels**

To match pixels, several structure lights were used to assign characteristic value to every pixel. Firstly, I use Gray code light pattern to generate an integer for every pixel. The detail is as follows. Let the projector project Gray code light pattern onto the 3D object and then let two cameras take several pictures, as depicted in Fig4.
 ![image](https://user-images.githubusercontent.com/67689632/200151761-2cd9aec0-b475-4787-95f2-5f71a7159696.png)

Fig 4
Six Gray code patterns were used and each one was used to generate one bit of the integer assigned to each pixel. When a Gray code pattern was projected on the object, we could see white and black stripes. For white stripes, the bit was set to 1. Otherwise, the bit was 0. After applying six Gray code patterns, each pixel had their six bits set and that formed a integer from 0 to 2^6. Every matching pixels must have the same integer. However, the Gray code stripe was too wide which would make several pixels shared the same integer value, as depicted in Fig5.
 ![image](https://user-images.githubusercontent.com/67689632/200151763-97064ab4-baa3-4e08-b635-7d5250b47305.png)

Fig 5
To address this problem, Phase-shift patterns are used to generate more finer value for each pixel. Unlike Gray code pattern where the light intensity changed abruptly, The light intensity of Phase-shift pattern changed smoothly and thus could give a continuous but periodic real number value to each pixel, as depicted in Fig 6.
 ![image](https://user-images.githubusercontent.com/67689632/200151766-8d3b977f-38f1-4004-a17b-a8f0c0f7a589.png)

Fig 6
By using the combed results generated from these two kinds of structured lights, we could match pixels in an accurate way.
2.1.3 generate 3D point coordinate values
After getting matched pixels, triangulation algorithm could be used to generate 3D point coordinate values, the result is shown in Fig 7
 ![image](https://user-images.githubusercontent.com/67689632/200151770-a90122b8-78a5-47e8-aa53-b4a6e92cfd82.png)

Fig 7

**2.2 generate surface topology from these 3D points**

After getting 3D points coordinate value, the surface topology spanned by these points can be generated by using Point Cloud Library (PCL). The final result is shown in Fig 8.
 ![image](https://user-images.githubusercontent.com/67689632/200151773-881729a3-eb40-4330-9c73-9612e6200bd0.png)

Fig 8

File description:
Main.cpp is the entry point of this software.
CamCalib.cpp is used to generate some parameters which can then be used to do 3D rectification.
CalPhase.cpp is used to generate characteristic value for each pixel.
Propointcloud.cpp is used to filter some noise of the point could and generate the surface structure of the 3D points.
