#include "MOOS/libMOOS/MOOSLib.h"
#include "MBUtils.h"
#include <Eigen/LU>
#include <boost/date_time/posix_time/posix_time.hpp>


using namespace Eigen;
using namespace std;

// format for inputting a matrix from a config file:
// mat_name = a,b,c;d,e,f;g,h,i
MatrixXd read_matrix(string str){
  MatrixXd m;
  vector<string> rows_vec = parseString(str,";"); //rows_vec[0] = "a,b,c"
  for(int i=0; i<rows_vec.size(); i++){
    vector<string> row_vals_vec = parseString(rows_vec[i],","); //row_vals_vec = "a"
    for(int j=0; j<row_vals_vec.size(); j++){
      if(i==0 && j ==0)
	m.resize(rows_vec.size(),row_vals_vec.size());
      m(i,j)=atof(row_vals_vec[j].c_str());
    }
  }
  return m;
}
  
int64_t time_in_ms(){
  boost::posix_time::ptime p(boost::posix_time::microsec_clock::universal_time());
  return (int64_t)(p.time_of_day().total_milliseconds());
}
