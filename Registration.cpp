#include "Registration.h"

#define SAVECURVATURE

#if 1
void ComputePriCur(void)
{
  const char* pcdfilename1 = "../mydata/output/pnts3D_pcd_filtered1.pcd";
  const char* pcdfilename2 = "../mydata/output/pnts3D_pcd_filtered2.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr input_cloudNormals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr target_cloudNormals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr input_cloudPriCur(new pcl::PointCloud<pcl::PrincipalCurvatures>);
  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr target_cloudPriCur(new pcl::PointCloud<pcl::PrincipalCurvatures>);
  
  if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcdfilename1, *input_cloud) == -1)
  {
    cout << "can not read file pnts3D_pcd_filtered1.pcd" <<endl;
    return ;
  }
  if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcdfilename2, *target_cloud) == -1)
  {
    cout << "can not read file pnts3D_pcd_filtered2.pcd" <<endl;
    return ;
  }
  
  //Computer the normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normals_estimation;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  normals_estimation.setSearchMethod(tree);
  normals_estimation.setRadiusSearch(0.003);
  normals_estimation.setInputCloud(input_cloud);
  normals_estimation.compute(*input_cloudNormals);
  normals_estimation.setInputCloud(target_cloud);
  normals_estimation.compute(*target_cloudNormals);
  
  //compute the principal curvatures
  pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> 
  Principal_curvatures_estimation;
  Principal_curvatures_estimation.setSearchMethod(tree);
  Principal_curvatures_estimation.setRadiusSearch(0.003);
  Principal_curvatures_estimation.setInputCloud(input_cloud);
  Principal_curvatures_estimation.setInputNormals(input_cloudNormals);
  Principal_curvatures_estimation.compute(*input_cloudPriCur);
  
  Principal_curvatures_estimation.setInputCloud(target_cloud);
  Principal_curvatures_estimation.setInputNormals(target_cloudNormals);
  Principal_curvatures_estimation.compute(*target_cloudPriCur);
  
  cout << "input_cloudPriCur size" <<  input_cloudPriCur->size() << endl;
 // cout << "target_cloudPriCur size" << target_cloudPriCur->size() << endl;
  
  cout << "input_cloudPriCur point[0] : " << input_cloudPriCur->points[1]<< endl;
 // cout << "input_cloudPriCur point[0] : " << input_cloudPriCur->points[1].principal_curvature_x<< endl;
  
  
#ifdef SAVECURVATURE
  const char* curfilename = "../mydata/output/principal_curvatures2.txt";
  
  FILE* fp = fopen(curfilename, "wt");
  for(int i = 0; i < target_cloudPriCur->size(); i++)
  {
    fprintf(fp, "%f   %f   %f   %f   %f \n", 
	    target_cloudPriCur->points[i].principal_curvature_x, 
	    target_cloudPriCur->points[i].principal_curvature_y, 
	    target_cloudPriCur->points[i].principal_curvature_z,
	    target_cloudPriCur->points[i].pc1,
	    target_cloudPriCur->points[i].pc2
 	  );
  }
  fclose(fp);
#endif  
}

#endif

#if 1
// 正态分布进行陪准
void pointcloud_registration(void)
{
  const char* pcdfilename1 = "../mydata/output/pcd.pcd";
  const char* pcdfilename2 = "../mydata/output/pcd.pcd";
  const char*  plyfilename1 = "../mydata/output/pcdPly.ply";
  const char*  plyfilename2 = "../mydata/output/pcdPly.ply";
  pcl::PCLPointCloud2 cloudPcd1, cloudPcd2;
  if(pcl::io::loadPCDFile(pcdfilename1, cloudPcd1) < 0)
  {
    cout << "can not open pcdfile 1!!!" <<endl;
    return ;
  }
  if(pcl::io::loadPCDFile(pcdfilename2, cloudPcd2) < 0)
  {
    cout << "can not open pcdfile 2!!!" <<endl;
    return ;
  }
  
  
  
  pcl::PLYWriter plyWriter;
  plyWriter.write(plyfilename1, cloudPcd1, Eigen::Vector4f::Zero(), 
    Eigen::Quaternionf::Identity(), false, true );
  plyWriter.write(plyfilename2, cloudPcd2, Eigen::Vector4f::Zero(), 
    Eigen::Quaternionf::Identity(), false, true );
  
#if 0
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcdfilename1, *target_cloud) == -1)
  {
    cout << "can not read file pnts3D_pcd_filtered1.pcd" <<endl;
    return ;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcdfilename2, *input_cloud) == -1)
  {
    cout << "can not read file pnts3D_pcd_filtered2.pcd" <<endl;
    return ;
  } 
  
  
  cout << "before approximate_voxel_filter " << input_cloud->size() << " data pionts " <<endl;
  
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(target_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(1.9,2.2);// model21 pass.setFilterLimits(1.45,1.9);
  pass.setFilterLimitsNegative(false);
  pass.filter(*target_cloud);
  
  pass.setInputCloud(input_cloud); 
  pass.filter(*input_cloud);
  
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
#if 1
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize(0.002, 0.002, 0.002);
  approximate_voxel_filter.setInputCloud(input_cloud);
  approximate_voxel_filter.filter(*filtered_cloud);
#endif
//   pcl::VoxelGrid< pcl::PointXYZ > sor;
//   sor.setInputCloud(input_cloud);
//   sor.setLeafSize(0.002f, 0.002f, 0.002f); // model40:0.002f, 0.002f, 0.003f; 0.0002f, 0.0002f, 0.0005f
//                                            // model62 0.0015f, 0.0015f, 0.002f
//   sor.filter(*filtered_cloud);
  
  
  cout << "after approximate_voxel_filter " << filtered_cloud->size() << "data points" <<endl;
  
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>  ndt;
  ndt.setTransformationEpsilon(0.005);
  ndt.setStepSize(0.005);
  ndt.setResolution(0.005);
  ndt.setMaximumIterations(500);
  ndt.setInputSource(filtered_cloud);
  ndt.setInputTarget(target_cloud);
  
  Eigen::AngleAxisf init_rotation(0.05931, Eigen::Vector3f::UnitZ());
  Eigen::Translation3f init_translation(0, 0, 0.3);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
 // ndt.align(*output_cloud, init_guess);
  ndt.align(*output_cloud);
  pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());
  
  
  /****************visualization************************/
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_final(new pcl::visualization::PCLVisualizer ("3D viewer"));
  viewer_final->setBackgroundColor(0,0,0);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud,255,0,0);
  viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target_cloud");
  viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(output_cloud,0,255,0);
  viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output_cloud");
  viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output_cloud");
  
  viewer_final->addCoordinateSystem(1.0);
  viewer_final->initCameraParameters();
  
  while(!viewer_final->wasStopped())
  {
     viewer_final->spinOnce(100);
  }
#endif
}
#endif

#if 0
void sac_ia(void)
{
  const char* pcdfilename1 = "../mydata/output/pnts3D_pcd_filtered1.pcd";
  const char* pcdfilename2 = "../mydata/output/pnts3D_pcd_filtered2.pcd";
  const char* result = "../mydata/output/result_sac12.pcd";
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tarCloudFil(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr inCloudFil(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr tarNormals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr inNormals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr tarFeatures(new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr inFeatures(new pcl::PointCloud<pcl::FPFHSignature33>);
  
  
  if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcdfilename1, *target_cloud) == -1)
  {
    cout << "can not read file pnts3D_pcd_filtered1.pcd" <<endl;
    return ;
  }

  if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcdfilename2, *input_cloud) == -1)
  {
    cout << "can not read file pnts3D_pcd_filtered2.pcd" <<endl;
    return ;
  }
  
  // VoxelGrid Filter
  pcl::VoxelGrid< pcl::PointXYZ > sor;
  sor.setInputCloud(target_cloud);
  sor.setLeafSize(0.005f, 0.005f, 0.005f);                                        
  sor.filter(*tarCloudFil);
  sor.setInputCloud(input_cloud);                                     
  sor.filter(*inCloudFil);
  
  //compute normal
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
 // n.setKSearch(30); // 20
  n.setRadiusSearch(0.002);
  n.setInputCloud(tarCloudFil);
  n.compute(*tarNormals);
  n.setInputCloud(inCloudFil);
  n.compute(*inNormals);
  
  //compute  local features
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>  fpfh_est;
  fpfh_est.setInputCloud(tarCloudFil);
  fpfh_est.setInputNormals(tarNormals);
  fpfh_est.setSearchMethod(tree);
  fpfh_est.setRadiusSearch(0.002f); // 
  fpfh_est.compute(*tarFeatures);
  fpfh_est.setInputCloud(inCloudFil);
  fpfh_est.setInputNormals(inNormals);
  fpfh_est.setSearchMethod(tree);
  fpfh_est.compute(*inFeatures);
  
  pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
  Eigen::Matrix4f final_transformation;
  sac_ia.setInputCloud(inCloudFil);
  sac_ia.setSourceFeatures(inFeatures);
  sac_ia.setInputTarget(tarCloudFil);
  sac_ia.setTargetFeatures(tarFeatures);
  sac_ia.setMaximumIterations(10000);
  sac_ia.setMinSampleDistance(3);
  sac_ia.setMaxCorrespondenceDistance(10000);
  pcl::PointCloud<pcl::PointXYZ> finalcloud;
  sac_ia.align(finalcloud);
  sac_ia.getCorrespondenceRandomness();
  
  Eigen::Matrix4f  init_transform = sac_ia.getFinalTransformation();
  pcl::transformPointCloud(*input_cloud, *input_cloud, init_transform);
  pcl::PointCloud<pcl::PointXYZ> final = *target_cloud;
  final +=*input_cloud;
  
  cout << init_transform <<endl;
  pcl::io::savePCDFileASCII(result, final); 
  
#if 0
  //ICP + LM
  pcl::transformPointCloud(*inCloudFil, *inCloudFil, init_transform);
  pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);
  
  src = input_cloud;
  tgt = target_cloud;
  
  //compute normals
  pcl::PointCloud<pcl::PointNormal>::Ptr pointsWithNormalSrc(new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr pointsWithNormalTgt(new pcl::PointCloud<pcl::PointNormal>);
  
  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal>  norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
  norm_est.setSearchMethod(tree2);
  norm_est.setKSearch(30);
  norm_est.setInputCloud(src);
  norm_est.compute(*pointsWithNormalSrc);
  pcl::copyPointCloud(*src, *pointsWithNormalSrc);
  norm_est.setInputCloud(tgt);
  norm_est.compute(*pointsWithNormalTgt);
  pcl::copyPointCloud(*tgt, *pointsWithNormalTgt);
  
  //icp + lm
  pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;
  MyPointRepresentation point_representation;
  reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));
  reg.setMaximumIterations(10000); //最大迭代次数
  reg.setMaxCorrespondenceDistance(0.015);
  reg.setTransformationEpsilon(1e-16);
  reg.setEuclideanFitnessEpsilon(0.0000002);   
  // reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));  
  reg.setInputSource(pointsWithNormalSrc);  
  reg.setInputTarget(pointsWithNormalTgt); 
 

  Eigen::Matrix4f Ti = Eigen::Matrix4f, prev;
  
#endif
  
  
  
}
#endif

#if 1
// icp
void hightaccuracy(void)
{
  /**************first registration*******************/
  const char* pcdfilename1 = "../mydata/output/pnts3D_pcd_filtered2.pcd";  // R T
  const char* pcdfilename2 = "../mydata/output/pnts3D_pcd_filtered1.pcd";  // 不动
  const char* regresult12 = "../mydata/output/result_pcd12.pcd";           // 保存结果
  
 /*****************second registration***********************/
//  const char* pcdfilename1 = "../mydata/output/result_pcd12.pcd";
//  const char* pcdfilename2 = "../mydata/output/pnts3D_pcd_filtered2.pcd";
//   const char* regresult123 = "../mydata/output/result_pcd127.pcd";
 
 /*****************three registration***********************/
//  const char* pcdfilename1 = "../mydata/output/pnts3D_pcd_filtered6.pcd";
//  const char* pcdfilename2 = "../mydata/output/result_pcd127.pcd";
//  const char* regresult1234 = "../mydata/output/result_pcd1267.pcd";
 
 /*****************four registration***********************/
//  const char* pcdfilename1 = "../mydata/output/pnts3D_pcd_filtered5.pcd";
//  const char* pcdfilename2 = "../mydata/output/result_pcd1267.pcd";
//  const char* regresult12345 = "../mydata/output/result_pcd12567.pcd";
 
 
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcdfilename1, *input_cloud) == -1)
  {
    cout << "can not read file pnts3D_pcd_filtered1.pcd" <<endl;
    return ;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcdfilename2, *target_cloud) == -1)
  {
    cout << "can not read file pnts3D_pcd_filtered2.pcd" <<endl;
    return ;
  }
  
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(target_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(1.85,2.2);// model21 pass.setFilterLimits(1.45,1.9);
  pass.setFilterLimitsNegative(false);
  pass.filter(*target_cloud);
  
  pass.setInputCloud(input_cloud); 
  pass.filter(*input_cloud);
  
  
  std::cout << "before: " << target_cloud->width*target_cloud->height << "data points" <<endl;
  
#if 1
  pcl::VoxelGrid< pcl::PointXYZ > sor;
  sor.setLeafSize(0.003f, 0.003f, 0.003f);
  sor.setInputCloud(target_cloud);                                      // model62 0.0015f, 0.0015f, 0.002f
  sor.filter(*target_cloud);
  sor.setInputCloud(input_cloud);                                      // model62 0.0015f, 0.0015f, 0.002f
  sor.filter(*input_cloud);
#endif
  
  std::cout << "after: " << target_cloud->width*target_cloud->height << "data points" <<endl;
  
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(input_cloud);
  icp.setInputTarget(target_cloud);  // 不动
  icp.setMaximumIterations(200); //最大迭代次数
  icp.setMaxCorrespondenceDistance(0.18);
  icp.setTransformationEpsilon(1e-12);
  icp.setEuclideanFitnessEpsilon(0.001);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
  icp.align(*final);

  
   /****************visualization************************/
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_final(new pcl::visualization::PCLVisualizer ("3D viewer"));
   viewer_final->setBackgroundColor(0,0,0);
  
   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud,255,0,0);
   viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target_cloud");
   viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud");
  
   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(final,0,255,0);
   viewer_final->addPointCloud<pcl::PointXYZ>(final, output_color, "output_cloud");
   viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output_cloud");
   
   viewer_final->addCoordinateSystem(1.0);
   viewer_final->initCameraParameters();
  
    while(!viewer_final->wasStopped())
    {
      viewer_final->spinOnce(100);
    }

  
    for(int i=0; i < target_cloud->size(); i++)
    {
      final->push_back(target_cloud->points[i]);
    }
  
    cout << "input_cloud->size" << input_cloud->size() << "points" <<endl;
    cout << "target_cloud->size" << target_cloud->size() << "points" <<endl;
    cout << "final->size" << final->size() << "points" <<endl;
    cout << icp.getFinalTransformation() << endl;
    
    pcl::io::savePCDFileASCII(regresult12, *final);
  

}

#endif

#if 1
// icp normals
void pairAlign(void)
{
    const char* pcdfilename1 = "../mydata/output/pnts3D_pcd_filtered1.pcd";
    const char* pcdfilename2 = "../mydata/output/pnts3D_pcd_filtered2.pcd";
    const char* regresult12 = "../mydata/output/result_pair12.pcd";
 
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcdfilename1, *input_cloud) == -1)
    {
      cout << "can not read file pnts3D_pcd_filtered1.pcd" <<endl;
      return ;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcdfilename2, *target_cloud) == -1)
    {
      cout << "can not read file pnts3D_pcd_filtered2.pcd" <<endl;
      return ;
    }
    
    std::cout << "before: " << target_cloud->width*target_cloud->height << "data points" <<endl;
    
#if 1
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(target_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(1.9,2.18);// model21 pass.setFilterLimits(1.45,1.9);
    pass.setFilterLimitsNegative(false);
    pass.filter(*target_cloud);
    
    pass.setInputCloud(input_cloud); 
    pass.filter(*input_cloud);
#endif
    
    pcl::VoxelGrid< pcl::PointXYZ > sor;
    sor.setLeafSize(0.004f, 0.004f, 0.004f); 
    sor.setInputCloud(target_cloud);                                  
    sor.filter(*target_cloud);
    sor.setInputCloud(input_cloud);                                  
    sor.filter(*input_cloud);   
    
    std::cout << "after: " << target_cloud->width*target_cloud->height << "data points" <<endl;
    
    pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_tgt(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr reg_result(new pcl::PointCloud<pcl::PointNormal>);
    
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> norm_est;  
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>()); 
    norm_est.setSearchMethod(tree);  
    norm_est.setKSearch(30);  
    norm_est.setInputCloud(input_cloud);  
    norm_est.compute(*points_with_normals_src);  
    pcl::copyPointCloud(*input_cloud, *points_with_normals_src);  
    norm_est.setInputCloud(target_cloud);  
    norm_est.compute(*points_with_normals_tgt);  
    pcl::copyPointCloud(*target_cloud, *points_with_normals_tgt); 
    
    pcl::IterativeClosestPointNonLinear< pcl::PointNormal, pcl::PointNormal> reg;  
    reg.setMaximumIterations(700); //最大迭代次数
    reg.setMaxCorrespondenceDistance(0.20);
    reg.setTransformationEpsilon(1e-12);
    reg.setEuclideanFitnessEpsilon(0.000001);   
   // reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));  
    reg.setInputSource(points_with_normals_src);  
    reg.setInputTarget(points_with_normals_tgt); 
    reg.align(*reg_result);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::PCDReader reader;
    pcl::io::savePCDFileASCII("../mydata/output/reg_result_points.pcd", *reg_result);  
    reader.read("../mydata/output/reg_result_points.pcd", *final);
    
    cout << reg.getFinalTransformation() << endl;
    
    
      /****************visualization************************/
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_final(new pcl::visualization::PCLVisualizer ("3D viewer"));
   viewer_final->setBackgroundColor(0,0,0);
  
   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud,255,0,0);
   viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target_cloud");
   viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud");
  
   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(final,0,255,0);
   viewer_final->addPointCloud<pcl::PointXYZ>(final, output_color, "output_cloud");
   viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output_cloud");
   
   viewer_final->addCoordinateSystem(1.0);
   viewer_final->initCameraParameters();
  
    while(!viewer_final->wasStopped())
    {
      viewer_final->spinOnce(100);
    }
    
    for(int i=0; i < target_cloud->size(); i++)
    {
      final->push_back(target_cloud->points[i]);
    }
      
     pcl::io::savePCDFileASCII(regresult12, *final); 
}


#endif

#if 0

float model_ss_ (0.005f);
float scene_ss_ (0.005f);
float rf_rad_ (0.015f);
float descr_rad_ (0.005f);
float cg_size_ (0.01f);
float cg_thresh_ (5.0f);

bool use_hough_ (true);
bool show_keypoints_ (true);
bool show_correspondences_ (true);


void hough(void)
{
  const char* pcdfilename1 = "../mydata/output/pnts3D_pcd_filtered1.pcd";
  const char* pcdfilename2 = "../mydata/output/pnts3D_pcd_filtered2.pcd";
  
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  model (new pcl::PointCloud<pcl::PointXYZRGBA> ());  //模型点云                            
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  modelKeyPoints (new pcl::PointCloud<pcl::PointXYZRGBA> ()); //模型点云关键点
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  scene (new pcl::PointCloud<pcl::PointXYZRGBA> ());   //目标点云
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  sceneKeyPoints (new pcl::PointCloud<pcl::PointXYZRGBA> ());  //目标点云关键点
  pcl::PointCloud<pcl::Normal>::Ptr modelNormals (new  pcl::PointCloud<pcl::Normal> ());  //法线
  pcl::PointCloud<pcl::Normal>::Ptr sceneNormals (new  pcl::PointCloud<pcl::Normal> ());
  pcl::PointCloud<pcl::SHOT352>::Ptr modelDescriptors (new pcl::PointCloud<pcl::SHOT352> ());
  pcl::PointCloud<pcl::SHOT352>::Ptr sceneDescriptors (new pcl::PointCloud<pcl::SHOT352> ());
  
  if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(pcdfilename1, *model) == -1)
  {
    cout << "can not read file pnts3D_pcd_filtered1.pcd" <<endl;
    return ;
  }
  if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(pcdfilename2, *scene) == -1)
  {
    cout << "can not read file pnts3D_pcd_filtered2.pcd" <<endl;
    return ;
  }
  
  pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> norm_est; 
  norm_est.setKSearch (30);       //设置K邻域搜索阀值为10个点
  norm_est.setInputCloud (model);  //设置输入点云
  norm_est.compute (*modelNormals);   //计算点云法线
  norm_est.setInputCloud (scene);   
  norm_est.compute (*sceneNormals);
  
    //均匀采样点云并提取关键点
  pcl::UniformSampling<pcl::PointXYZRGBA> uniform_sampling;
  uniform_sampling.setInputCloud (model);  //输入点云
  uniform_sampling.setRadiusSearch (model_ss_);   //设置半径
  
 // uniform_sampling.filter(*modelKeyPoints);   //滤波
  pcl::PointCloud<int> keypointIndices1;
  uniform_sampling.compute(keypointIndices1);
  pcl::copyPointCloud(*model, keypointIndices1.points, *modelKeyPoints);
  
  std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << modelKeyPoints->size () << std::endl;
  uniform_sampling.setInputCloud (scene);
  uniform_sampling.setRadiusSearch (scene_ss_);
  
 // uniform_sampling.filter (*sceneKeyPoints);
  pcl::PointCloud<int> keypointIndices2;
  uniform_sampling.compute(keypointIndices2);
  pcl::copyPointCloud(*scene, keypointIndices2.points, *sceneKeyPoints);
  
  std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << sceneKeyPoints->size () << std::endl;
  
    //为关键点计算描述子
  pcl::SHOTEstimationOMP<pcl::PointXYZRGBA, pcl::Normal, pcl::SHOT352> descr_est;
  descr_est.setRadiusSearch (descr_rad_);  //设置搜索半径

  descr_est.setInputCloud (modelKeyPoints);  //输入模型的关键点
  descr_est.setInputNormals (modelNormals);  //输入模型的法线
  descr_est.setSearchSurface (model);         //输入的点云
  descr_est.compute (*modelDescriptors);     //计算描述子

  descr_est.setInputCloud (sceneKeyPoints);  //同理
  descr_est.setInputNormals (sceneNormals);
  descr_est.setSearchSurface (scene);
  descr_est.compute (*sceneDescriptors);
  
  //  使用Kdtree找出 Model-Scene 匹配点
  pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences ());
  pcl::KdTreeFLANN<pcl::SHOT352> match_search;   //设置配准的方法
  match_search.setInputCloud (modelDescriptors);  //输入模板点云的描述子
  //每一个场景的关键点描述子都要找到模板中匹配的关键点描述子并将其添加到对应的匹配向量中。
  for (size_t i = 0; i < sceneDescriptors->size (); ++i)
  {
    std::vector<int> neigh_indices (1);   //设置最近邻点的索引
    std::vector<float> neigh_sqr_dists (1); //申明最近邻平方距离值
    if (!pcl_isfinite (sceneDescriptors->at (i).descriptor[0])) //忽略 NaNs点
    {
      continue;
    }
    int found_neighs = match_search.nearestKSearch (sceneDescriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
//sceneDescriptors->at (i)是给定点云 1是临近点个数 ，neigh_indices临近点的索引  neigh_sqr_dists是与临近点的索引

    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) // 仅当描述子与临近点的平方距离小于0.25（描述子与临近的距离在一般在0到1之间）才添加匹配
    {
//neigh_indices[0]给定点，  i  是配准数  neigh_sqr_dists[0]与临近点的平方距离
      pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      model_scene_corrs->push_back (corr); //把配准的点存储在容器中
       
    }
  }
  std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

  
    //  实际的配准方法的实现

  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;

  std::vector<pcl::Correspondences> clustered_corrs;
  
    //  使用 Hough3D算法寻找匹配点
  if (use_hough_)
  {
    //
    //  Compute (Keypoints) Reference Frames only for Hough

    //计算参考帧的Hough（也就是关键点）
    pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf (new pcl::PointCloud<pcl::ReferenceFrame> ());
    pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf (new pcl::PointCloud<pcl::ReferenceFrame> ());
    //特征估计的方法（点云，法线，参考帧）
    pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::ReferenceFrame> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (rf_rad_);   //设置搜索半径

    rf_est.setInputCloud (modelKeyPoints);  //模型关键点
    rf_est.setInputNormals (modelNormals); //模型法线
    rf_est.setSearchSurface (model);    //模型
    rf_est.compute (*model_rf);      //模型的参考帧   

    rf_est.setInputCloud (sceneKeyPoints);  //同理
    rf_est.setInputNormals (sceneNormals);
    rf_est.setSearchSurface (scene);
    rf_est.compute (*scene_rf);

    //  Clustering聚类的方法


    //对输入点与的聚类，以区分不同的实例的场景中的模型 

    pcl::Hough3DGrouping<pcl::PointXYZRGBA, pcl::PointXYZRGBA, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
    clusterer.setHoughBinSize (cg_size_);//霍夫空间设置每个bin的大小
    clusterer.setHoughThreshold (cg_thresh_);//阀值
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (modelKeyPoints);
    clusterer.setInputRf (model_rf);   //设置输入的参考帧
    clusterer.setSceneCloud (sceneKeyPoints);
    clusterer.setSceneRf (scene_rf);
    clusterer.setModelSceneCorrespondences (model_scene_corrs);//model_scene_corrs存储配准的点

    //clusterer.cluster (clustered_corrs);辨认出聚类的对象
    clusterer.recognize (rototranslations, clustered_corrs);
  }
  else // Using GeometricConsistency  或者使用几何一致性性质

  {
    pcl::GeometricConsistencyGrouping<pcl::PointXYZRGBA, pcl::PointXYZRGBA> gc_clusterer;
    gc_clusterer.setGCSize (cg_size_);   //设置几何一致性的大小
    gc_clusterer.setGCThreshold (cg_thresh_); //阀值

    gc_clusterer.setInputCloud (modelKeyPoints);
    gc_clusterer.setSceneCloud (sceneKeyPoints);
    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //gc_clusterer.cluster (clustered_corrs);
    gc_clusterer.recognize (rototranslations, clustered_corrs);
  }
    //输出的结果  找出输入模型是否在场景中出现

  std::cout << "Model instances found: " << rototranslations.size () << std::endl;
  for (size_t i = 0; i < rototranslations.size (); ++i)
  {
    std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
    std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;
  
        // 打印处相对于输入模型的旋转矩阵与平移矩阵

    Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
    Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

    printf ("\n");
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    printf ("\n");
    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  }
  
  //可视化
  pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
  viewer.addPointCloud (scene, "scene_cloud");//可视化场景点云

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr off_scene_model (new pcl::PointCloud<pcl::PointXYZRGBA> ());
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr off_scene_model_keypoints (new pcl::PointCloud<pcl::PointXYZRGBA> ());
  
  if (show_correspondences_ || show_keypoints_)  //可视化配准点
  {
    //  We are translating the model so that it doesn't end in the middle of the scene representation

    //就是要对输入的模型进行旋转与平移，使其在可视化界面的中间位置
    pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
    pcl::transformPointCloud (*modelKeyPoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));  //因为模型的位置变化了，所以要对特征点进行变化

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> off_scene_model_color_handler (off_scene_model, 255, 255, 128);  //对模型点云上颜色

    viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
  }
  if (show_keypoints_)   //可视化关键点
  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> scene_keypoints_color_handler (sceneKeyPoints, 0, 0, 255);  //对场景中的点云上为绿色
    viewer.addPointCloud (sceneKeyPoints, scene_keypoints_color_handler, "scene_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
    viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
  }

   for (size_t i = 0; i < rototranslations.size (); ++i)
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr rotated_model (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);//把model转化为rotated_model

//    <Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >   rototranslations是射影变换矩阵

    std::stringstream ss_cloud; 
    ss_cloud << "instance" << i;

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> rotated_model_color_handler (rotated_model, 255, 0, 0);
    viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

    if (show_correspondences_)   //显示配准连接

    {
      for (size_t j = 0; j < clustered_corrs[i].size (); ++j)
      {
        std::stringstream ss_line;
        ss_line << "correspondence_line" << i << "_" << j;
        pcl::PointXYZRGBA & model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
        pcl::PointXYZRGBA & scene_point = sceneKeyPoints->at (clustered_corrs[i][j].index_match);

        //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
        viewer.addLine<pcl::PointXYZRGBA, pcl::PointXYZRGBA> (model_point, scene_point, 0, 255, 0, ss_line.str ());
      }
    }
  }


  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  } 
}
#endif






