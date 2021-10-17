#include <iostream>
#include <vector>
#include <string>

#include <sstream>
#include <fstream>

#include <Eigen/Dense>
#include <cmath>

using namespace std;

struct gridValueStruct{
    int gridValue;
    int count;

    bool operator == (const int &now){
        return (this->gridValue == now);
    }
};

void read_depth_image(Eigen::MatrixXf& fdepth, int rows, int cols){
    std::string str, strs;
	std::vector<double> myDouble;
    double value;

    ifstream inFile;
	inFile.open("/home/ncslaber/110-1/211009_allLibrary/front-right/syn_rosbag/depth-30.csv");
    // inFile.open("/home/ncslaber/110-1/211010_scanMatching/test.csv");
	if (inFile.is_open())
	{
        cout << "reading depthimageValue.csv" << endl;
        while ( getline(inFile, strs) )
		{
            std::stringstream ss(strs);

            while( getline(ss, str, ',') )
            {
                value = atof(str.c_str());
                myDouble.push_back(value);
            }
        }
    }
    else 
	    std::cout<< "error opening depthimageValue.csv" << std::endl;
	inFile.close();
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            fdepth(i,j) = myDouble[ cols*i+j ];
    // cout << fdepth ;
}
double range_max=7000,range_min=450;
double use_point( vector<struct gridValueStruct> vec_gridValueStruct, bool flag ){
    int max=0, tmp, tmpV;
    double value;
    // for (int i=vec_gridValueStruct.size()-1;i>=0;i--)
    for (int i=0;i<vec_gridValueStruct.size();i++)
    {
        // cout<<i<<' '<<endl;
        tmp = vec_gridValueStruct[i].count;
        tmpV = vec_gridValueStruct[i].gridValue;
        if (max < tmp) //&& tmpV != 140)
        {
            max = tmp;
            value = tmpV;
        }
        
    }

    if (max<4)
        value = (int)( (float)range_max/50+0.5 );
    
    // std::ofstream inFile;
    // inFile.open("/home/ncslaber/transformed_laserScan_number-20.csv", std::ios::out | std::ios_base::app);
    // if(!inFile)     
    //     std::cout << "Can't open file!\n";
    // else
    //   std::cout<<"File open successfully!\n";
    // if (flag)
    //     inFile << max ;
    // else
    //     inFile << max << ',';
    // inFile.close();

    // std::ofstream in2File;
    // in2File.open("/home/ncslaber/transformed_laserScan_gridValue-20.csv", std::ios::out | std::ios_base::app);
    // if(!in2File)     
    // std::cout << "Can't open file!\n";
    // else
    //   std::cout<<"File open successfully!\n";

    // if (flag)
    //     in2File << value ;
    // else
    //     in2File << value << ',';
    // in2File.close();
    
    return value;
}

int main(){
    int rows=480, cols=640;
    // int rows=2, cols=3;
    Eigen::MatrixXf fdepth(rows,cols);
    double scan[cols],r;
    

    read_depth_image(fdepth,rows,cols);
    vector<struct gridValueStruct> vector_diffLayer[cols];
    vector<struct gridValueStruct>::iterator it;
    struct gridValueStruct gv;
    
    int scan_height_=20;
    const int offset = (int)(rows/2 - scan_height_/2);

    double data;
    // cout<<A<<"here";
    //
    double cx_d = 320.6562194824219;
    double cy_d = 241.57083129882812;
    double fx_d = 384.31365966796875;
    double fy_d = 384.31365966796875;
    
    Eigen::ArrayXf npPointX = Eigen::ArrayXf::LinSpaced(cols, 0, cols-1);//.transpose() 
    npPointX = npPointX + cx_d;
    // npPointX = np.asarray(range(640))-cx_d
    Eigen::MatrixXf npPointMX( npPointX.matrix().asDiagonal() ); 
    // npPointX = np.diag(npPointX)
    npPointMX = fdepth*npPointMX / fx_d * (-1);
    // cout << npPointMX;

    Eigen::ArrayXf npPointY = Eigen::ArrayXf::LinSpaced(rows, 0, rows-1);//.transpose() 
    npPointY = npPointY - cy_d;
    // npPointY = np.asarray(range(480))-cy_d
    Eigen::MatrixXf npPointMY( npPointY.matrix().asDiagonal() ); 
    // npPointY = np.diag(npPointY)
    double theta = 0./180.*3.1415926;
    npPointMY = npPointMY*fdepth/ fy_d * (-1);
    npPointMY = npPointMY * cos(theta) + fdepth * sin(theta);
    npPointMY = npPointMY.array() + 410.0;
        // cout << npPointMY <<endl;
    // npPointMY = npPointMY.astype('float16')
    // npHeight = np.copy(npPointMY)
    Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> foo1 = npPointMY.array()<500; //
    Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> foo2 = npPointMY.array()>300; //
    Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> foo3 = npPointMY.array()!=410;
    Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic> height_layer = (foo1.array() * foo2.array()).matrix();
    height_layer = (height_layer.array() * foo3.array()).matrix();
        // cout<<height_layer;
    
    // cout << height_layer.array().any() << endl;
    // for(int v = 0; v < rows; ++v)
    // {
    //     if (height_layer.row(v).array().any()) //
    //     {
    //         for (int u = 0; u<cols; ++u) // Loop over each pixel in row
    //         {   
    //             if (height_layer(v,u) == true)
    //             {
    //                 cout<<"a";
    //             }
    //         }
    //     }
    // }
    
    // height_layer_tmp = np.logical_and(npHeight<500,npHeight>300)
    // height_layer = np.logical_and(height_layer_tmp,npHeight!=410)
    //
    // for(int v = offset; v < offset+scan_height_; ++v)
    for(int v = 0; v < rows; ++v)
    {
        // cout<<"a";
        if (height_layer.row(v).array().any()) //
        {
            for (int u = 0; u<cols; ++u) // Loop over each pixel in row
            {   
                if (height_layer(v,u) == true)
                {
                    r = fdepth(v,u); // r in mm
                    // cout << r <<endl;
                    // const bool range_check = range_min <= r && r <= range_max;

                    if (!(range_min <= r)){
                        r = 0;
                    }
                    else if(!(r <= range_max)){
                        r = range_max;
                    }
                    if (r != 0)
                    {
                        int now = (int)( (float)r/50+0.5 );
                        
                        it = std::find(vector_diffLayer[u].begin(), vector_diffLayer[u].end(), now);

                        if ( it != vector_diffLayer[u].end() )
                        {
                            // cout << "find same" <<endl;
                            it->count += 1;
                        }
                        else
                        {
                            // cout << "not same" <<endl;
                            gv.gridValue = now;
                            gv.count = 1;
                            vector_diffLayer[u].push_back( gv );
                        }
                    }
                }
            }
        }
        // cout<<endl;
    }

    for (int i = 0; i < cols; ++i)
    {
        if (i == cols-1)
            data = (use_point( vector_diffLayer[i], true )) * 0.05; // data in meter
        else
            data = (use_point( vector_diffLayer[i], false )) * 0.05; // data in meter
        if (data>1e-2)
            scan[i] = data;
        else
            scan[i] = 0;
        // cout << vector_diffLayer[i][i].count << endl;
    }

    std::ofstream newFile;
    newFile.open("/home/ncslaber/transformed_laserScan_depth-30_heightTest.csv", std::ios::out | std::ios::trunc);
    if(!newFile)     
      std::cout << "Can't open file!\n";
    else
      std::cout<<"File open successfully!!\n";
    
    for (int i=0; i<cols;i++) {
        // std::cout << *(scan+i) << std::endl;
        newFile << *(scan+i);
        if (i!=cols-1)
        {
            newFile << ',';
        }
    }
    newFile.close();

    return 0;
}