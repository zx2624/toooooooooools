#include <dirent.h>

#include <vector>
#include <string>
#include <algorithm>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGBL PointT;

pcl::PointCloud<PointT>::Ptr cloudPtr(new pcl::PointCloud<PointT>());

std::vector<std::string> load_dir_files(const char *dir)
{
    DIR *dp;
    struct dirent *ep;
    char filename[PATH_MAX];
    dp = opendir(dir);
    if(dp == NULL) LOG(ERROR)<< dir << " not exists";
    std::vector<std::string> files;
    while((ep = readdir(dp)))
    {
        if(ep->d_name[0] == '.')
            continue;
        sprintf(filename,"%s/%s",dir,ep->d_name);
        LOG(INFO)<<std::string(filename);
        files.push_back(std::string(filename));
    }
    closedir(dp);
    sort(files.begin(), files.end());
    return files;
}

void myLoadPCDFile(const std::string filenames,  pcl::PointCloud<PointT>::Ptr cloud)
{

	if(pcl::io::loadPLYFile(filenames, *cloud) == -1)
	{
		LOG(ERROR)<<"ERROR: cannot load pcd file\n";
		exit(1);
	}

	LOG(INFO)<<"cloud size: "<<cloud->size();
	*cloudPtr+=*cloud;
}


void toPly (char *argv)
{
	pcl::PLYWriter writer;
	writer.write(argv, *cloudPtr, true);
}
/* ---[ */

int main (int argc, char** argv)
{
	FLAGS_logtostderr=1;
	FLAGS_colorlogtostderr=1;
	google::InitGoogleLogging(argv[0]);

	// Parse the command line arguments for .pcd and .ply files
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
	std::vector<std::string> pcl_files;
	LOG(INFO) <<"pcl dir:" << argv[1];
	LOG(INFO) <<"ply file name:" << argv[2];
	pcl_files=load_dir_files(argv[1]);

	cloudPtr->clear();

	auto cnt=0;
	while (cnt < pcl_files.size())
	{
		myLoadPCDFile(pcl_files[cnt],cloud);
		cnt++;
	}
	// Convert to PLY and save
	toPly(argv[2]);

	return (0);
}
