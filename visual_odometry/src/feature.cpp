#include "../include/visual_odometry/feature.hpp"

void depthcomp(Mat& limg, Mat& rimg, Mat& depth){

}
void posecomp(vector<KeyPoint>& kp1, vector<KeyPoint>& kp2, vector<DMatch>& g_matches, calib_data& calib1, calib_data& calib2, Mat& R, Mat& t){
    vector<Point2f> point1, point2;

    for(DMatch i:g_matches){
        point1.push_back(kp1[i.queryIdx].pt);
        point2.push_back(kp2[i.trainIdx].pt);
    }
    
    Mat E;
    //Mat E = findEssentialMat(point1, point2, calib1.cam, cv::RANSAC, 0.999, 1.0, 1000, noArray());
    cout<<"match size - "<<g_matches.size()<<endl;
    //cout<<"Essential Matrix - "<<E.size()<<endl;
    recoverPose(point1, point2, calib1.cam, calib1.dist, calib2.cam, calib2.dist, E, R, t,cv::RANSAC,0.999, 1.0, noArray());

    
}

void fdetectMatch(Mat& limg1, Mat& rimg1, vector<KeyPoint>& kp, vector<KeyPoint>& kp2, vector<DMatch>& g_match){
 
    Mat ldesc;
    Mat rdesc;
    Mat limg,rimg;

    Ptr<CLAHE> clahe = createCLAHE();
    clahe->setClipLimit(4);
    clahe->setTilesGridSize(Size(16,16));
    
    clahe->apply(limg1, limg);
    clahe->apply(rimg1, rimg);
      
    //FEATURE DETECTOR

    Ptr<ORB> orb = ORB::create(500, 1.2, 16, 20, 2, 2, ORB::HARRIS_SCORE, 20, 15);
    orb->detectAndCompute(limg, noArray(), kp, ldesc);
	orb->detectAndCompute(rimg, noArray(), kp2, rdesc);

    ldesc.convertTo(ldesc, CV_32F);
    rdesc.convertTo(rdesc, CV_32F);
    for(int i=0;i<20;i++){
        cout<<"point "<<i<<" :";
        cout<<kp[i].pt.x<<", "<<kp[i].pt.y<<endl;
        cout<<kp2[i].pt.x<<", "<<kp2[i].pt.y<<endl;
    }
    //FEATURE MATCHER
    vector<DMatch> matches;
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
	matcher->match(ldesc, rdesc, matches);
    cout<<"hello  --as - - "<<rimg.size()<<endl;



    // Mat ldesc;
    // Mat rdesc;
    // vector<DMatch> matches;
    // Mat limgSlice, rimgSlice;
    // vector<KeyPoint> kp,kp2;

    // //FEATURE DETECTOR
    // Ptr<ORB> orb = ORB::create(500, 1.2, 16, 0, 2, 2, ORB::HARRIS_SCORE, 20, 15);

   	// Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);


    // for(int i=0; i<3; i++){
    //     limgSlice = limg(Range((i*80)+1, (i+1)*80),Range(20,300));
    //     rimgSlice = rimg(Range((i*80)+1, (i+1)*80),Range(20,300));
    //     orb->detectAndCompute(limgSlice, noArray(), kp, ldesc);
	//     orb->detectAndCompute(rimgSlice, noArray(), kp2, rdesc);
        
    //     orb->detectAndCompute(limgSlice, noArray(), kp, ldesc);
    // 	orb->detectAndCompute(rimgSlice, noArray(), kp2, rdesc);
    //     for(int j=0; j<kp.size();j++){
    //         kp[j].pt.y=kp[j].pt.y+(i*80)+1;
    //     }
    //     for(int j=0; j<kp2.size();j++){
    //         kp2[j].pt.y=kp2[j].pt.y+(i*80)+1;
    //     }
    //     kplimg.insert(kplimg.end(),kp.begin(),kp.end());
    //     kprimg.insert(kprimg.end(),kp2.begin(),kp2.end());
    //     ldesc.convertTo(ldesc, CV_32F);
    //     rdesc.convertTo(rdesc, CV_32F);
    //     vector<DMatch> matchesSlice;
	//     matcher->match(ldesc, rdesc, matchesSlice);
    //     matches.insert(matches.end(),matchesSlice.begin(),matchesSlice.end());
    // }
    

    

    //FEATURE MATCHER
    

    //OUTLIER REMOVAL
    auto min_max = minmax_element(matches.begin(), matches.end(),[](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
    cout<<"min_dist - "<<min_max.first->distance<<endl;
    cout<<"max_dist - "<<min_max.second->distance<<endl;
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;
    for(int i=0;i<ldesc.rows;i++){
        if(matches[i].distance <= max(2*min_dist,60.0)){
            g_match.push_back(matches[i]);
        }
    }
    
}

calib_data read_yaml2(const YAML::Node& node1, const YAML::Node& node2, const YAML::Node& node3){
	int rows1 = node1["rows"].as<int>();
    int cols1 = node1["cols"].as<int>();
	int rows2 = node2["rows"].as<int>();
    int cols2 = node2["cols"].as<int>();
	int rows3 = node3["rows"].as<int>();
    int cols3 = node3["cols"].as<int>();

	
	std::vector<double> data1 = node1["data"].as<std::vector<double>>();
	std::vector<double> data2 = node2["data"].as<std::vector<double>>();
	std::vector<double> data3 = node3["data"].as<std::vector<double>>();


	calib_data calb;
	//Mat cam(3, 3, CV_64F);
	//Mat dist(1, 5, CV_64F);
	for (int i = 0; i < rows1; ++i) {
        for (int j = 0; j < cols1; ++j) {
            calb.cam.at<double>(i, j) = data1[i * cols1 + j];
        }
    }
	for (int i = 0; i < rows2; ++i) {
        for (int j = 0; j < cols2; ++j) {
            calb.dist.at<double>(i, j) = data2[i * cols2 + j];
        }
    }
	for (int i = 0; i < rows3; ++i) {
        for (int j = 0; j < cols3; ++j) {
            calb.proj.at<double>(i, j) = data3[i * cols3 + j];
        }
    }
	return calb;
}