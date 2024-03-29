
#include "pointcloud_utils/io/frame_conversions.h"

//------------------------------------------------------------------------------------------------
// Class Constructor
//------------------------------------------------------------------------------------------------
WgsConversions::WgsConversions(){
	reference_lla[0] = 0 ; 
	reference_lla[1] = 0 ; 
	reference_lla[2] = 0 ; 
    reference_lla_flag = false ; 
  std::cout << "WgsConversions::WgsConversions" << std::endl;
}

//------------------------------------------------------------------------------------------------
// Class Destructor
//------------------------------------------------------------------------------------------------
WgsConversions::~WgsConversions(){
  std::cout << "WgsConversions::~WgsConversions" << std::endl;
}

//------------------------------------------------------------------------------------------------
// WgsConversions::updateReferenceLLA [Public]  --- update reference LLA the class uses 
//------------------------------------------------------------------------------------------------
void WgsConversions::updateReferenceLLA(double ref_lla[3], bool rad_or_deg){
    double rad2deg = 180.0 / M_PI;

    // bool rad_or_deg (1 --> deg, 0 --> rad)
    if(rad_or_deg)
    {
        reference_lla[0] = ref_lla[0]; 
        reference_lla[1] = ref_lla[1];
        reference_lla[2] = ref_lla[2];  
    }
    reference_lla[0] = rad2deg*ref_lla[0]; 
    reference_lla[1] = rad2deg*ref_lla[1];
    reference_lla[2] = rad2deg*ref_lla[2];  

    reference_lla_flag = true; 
}

void WgsConversions::updateReferenceLLA(double ref_lla[3]){
    // Assuming Degrees 
    reference_lla[0] = ref_lla[0]; 
    reference_lla[1] = ref_lla[1];
    reference_lla[2] = ref_lla[2];  
    reference_lla_flag = true;  
}

//------------------------------------------------------------------------------------------------
// WgsConversions::ned2enu [Public]  --- convert from (North,East,Down) to (East,North,Up)
//------------------------------------------------------------------------------------------------
bool WgsConversions::ned2enu(double enu[3],const double ned[3]){
    enu_nedFlip(enu,ned);
    return 1;
}

//------------------------------------------------------------------------------------------------
// WgsConversions::enu2ned [Public]  --- convert from (East,North,Up) to (North,East,Down)
//------------------------------------------------------------------------------------------------
bool WgsConversions::enu2ned(double ned[3],const double enu[3]){
    enu_nedFlip(ned,enu);
    return 1;
}

//------------------------------------------------------------------------------------------------
// WgsConversions::ned2lla [Public]  --- convert from (North,East,Down) to (Lat,Long,Alt)
//------------------------------------------------------------------------------------------------
bool WgsConversions::ned2lla(double lla[3], const double ned[3], const double ref_lla[3]){

    double enu[3];

    // Change to ENU
    enu_nedFlip(enu,ned) ;

    if(!enu2lla(lla,enu,ref_lla))
        return 0;

    return 1; 
}

bool WgsConversions::ned2lla(double lla[3], const double ned[3]){

	if(!reference_lla_flag)
		std::cout << "Attempted ned2lla..." << std::endl;
	    std::cout << "No reference LLA setup or inputted" << std::endl;
	    std::cout << "ned2lla failed" <<std::endl; 
		return 0; 

	if(!ned2lla(lla,ned,reference_lla))
		return 0; 

	return 1; 
}

//------------------------------------------------------------------------------------------------
// WgsConversions::lla2ned [Public]  --- convert from (Lat,Long,Alt) to (North,East,Down)
//------------------------------------------------------------------------------------------------
bool WgsConversions::lla2ned(double ned[3], const double lla[3], const double ref_lla[3]){

    double enu[3];

    // Go to ENU 
    if(!lla2enu(enu,lla,ref_lla))
        return 0;

    enu_nedFlip(ned,enu) ;

    return 1;  
}

bool WgsConversions::lla2ned(double lla[3], const double ned[3]){

	if(!reference_lla_flag)
		std::cout << "Attempted lla2ned..." << std::endl;
	    std::cout << "No reference LLA setup or inputted" << std::endl;
	    std::cout << "lla2ned failed" <<std::endl; 
		return 0; 

	if(!lla2ned(lla,ned,reference_lla))
		return 0; 

	return 1; 
}


//------------------------------------------------------------------------------------------------
// WgsConversions::enu2lla [Public]  --- convert from (East,North,Up) to (Lat,Long,Alt)
//------------------------------------------------------------------------------------------------
bool WgsConversions::enu2lla(double lla[3], const double enu[3], const double ref_lla[3]){

	double ref_xyz[3],diff_xyz[3],xyz[3],R[3][3],Rt[3][3];

	// First, calculate the xyz of reflat, reflon, refalt
	if (!lla2xyz(ref_xyz,ref_lla))
		return 0;

    rot3d(R, ref_lla[0], ref_lla[1]);

    transposeMatrix(Rt,R);

    matrixMultiply(diff_xyz,Rt,enu);

    xyz[0] = diff_xyz[0] + ref_xyz[0];
    xyz[1] = diff_xyz[1] + ref_xyz[1];
    xyz[2] = diff_xyz[2] + ref_xyz[2];

    if(!xyz2lla(lla,xyz))
    	return 0;

    return 1;
}

bool WgsConversions::enu2lla(double lla[3], const double enu[3]){

	if(!reference_lla_flag)
		std::cout << "Attempted enu2lla..." << std::endl;
	    std::cout << "No reference LLA setup or inputted" << std::endl;
	    std::cout << "enu2lla failed" <<std::endl; 
		return 0; 

	if(!enu2lla(lla,enu,reference_lla))
		return 0; 

	return 1; 
}

//------------------------------------------------------------------------------------------------
// WgsConversions::lla2enu [Public]  --- convert from (Lat,Long,Alt) to (East,North,Up)
//------------------------------------------------------------------------------------------------
bool WgsConversions::lla2enu(double enu[3], const double lla[3], const double ref_lla[3]){
  
	double xyz[3];

	if(!lla2xyz(xyz,lla))
		return 0;

	if(!xyz2enu(enu,xyz,ref_lla))
		return 0;

  return 1;
}

bool WgsConversions::lla2enu(double enu[3], const double lla[3]){

	if(!reference_lla_flag)
		std::cout << "Attempted lla2enu..." << std::endl;
	    std::cout << "No reference LLA setup or inputted" << std::endl;
	    std::cout << "lla2enu failed" <<std::endl; 
		return 0; 

	if(!lla2enu(enu,lla,reference_lla))
		return 0; 

	return 1; 
}

//------------------------------------------------------------------------------------------------
// WgsConversions::xyz2lla [Public]  --- convert from (ECEF X, ECEF Y, ECEF Z) to (Lat,Long,Alt)
//------------------------------------------------------------------------------------------------
bool WgsConversions::xyz2lla(double lla[3], const double xyz[3]){

    //This dual-variable iteration seems to be 7 or 8 times faster than
    //a one-variable (in latitude only) iteration.  AKB 7/17/95

    double A_EARTH = 6378137.0;
    double flattening = 1.0 / 298.257223563;
    double NAV_E2 = (2.0 - flattening) * flattening; // also e^2
    double rad2deg = 180.0 / M_PI;

    if ((xyz[0] == 0.0) & (xyz[1] == 0.0)) {
        lla[1] = 0.0;
    } else {
        lla[1] = atan2(xyz[1], xyz[0]) * rad2deg;
    }

    if ((xyz[0] == 0.0) & (xyz[1] == 0.0) & (xyz[2] == 0.0)) {
		std::cout << "WGS xyz at center of earth" << std::endl;
		return 0;
    } else {
        // Make initial lat and alt guesses based on spherical earth.
        double rhosqrd = xyz[0] * xyz[0] + xyz[1] * xyz[1];
        double rho = sqrt(rhosqrd);
        double templat = atan2(xyz[2], rho);
        double tempalt = sqrt(rhosqrd + xyz[2] * xyz[2]) - A_EARTH;
        double rhoerror = 1000.0;
        double zerror = 1000.0;
        
        int iter = 0; // number of iterations

        //      %  Newton's method iteration on templat and tempalt makes
        //      %   the residuals on rho and z progressively smaller.  Loop
        //      %   is implemented as a 'while' instead of a 'do' to simplify
        //      %   porting to MATLAB

        while ((abs(rhoerror) > 1e-6) | (abs(zerror) > 1e-6)) {
            double slat = sin(templat);
            double clat = cos(templat);
            double q = 1.0 - NAV_E2 * slat*slat;
            double r_n = A_EARTH / sqrt(q);
            double drdl = r_n * NAV_E2 * slat * clat / q; // d(r_n)/d(latitutde)

            rhoerror = (r_n + tempalt) * clat - rho;
            zerror = (r_n * (1.0 - NAV_E2) + tempalt) * slat - xyz[2];

            //          %             --                               -- --      --
            //          %             |  drhoerror/dlat  drhoerror/dalt | |  a  b  |
            //                        % Find Jacobian           |                       |=|        |
            //          %             |   dzerror/dlat    dzerror/dalt  | |  c  d  |
            //          %             --                               -- --      --

            double aa = drdl * clat - (r_n + tempalt) * slat;
            double bb = clat;
            double cc = (1.0 - NAV_E2)*(drdl * slat + r_n * clat);
            double dd = slat;

            //Apply correction = inv(Jacobian)*errorvector

            double invdet = 1.0 / (aa * dd - bb * cc);
            templat = templat - invdet * (+dd * rhoerror - bb * zerror);
            tempalt = tempalt - invdet * (-cc * rhoerror + aa * zerror);

            iter++;

            if (iter>20){
            	std::cout << "xyz2lla could not converge" << std::endl;
            	return 0;
            }
        }

        lla[0] = templat*rad2deg;
        lla[2] = tempalt;
    }
    return 1;
}

//------------------------------------------------------------------------------------------------
// WgsConversions::lla2xyz [Public]  --- convert from (Lat,Long,Alt) to (ECEF X, ECEF Y, ECEF Z)
//------------------------------------------------------------------------------------------------
bool WgsConversions::lla2xyz(double xyz[3], const double lla[3]){

	if ((lla[0] < -90.0) | (lla[0] > +90.0) | (lla[1] < -180.0) | (lla[1] > +360.0)){
		std::cout << "WGS lat or WGS lon out of range" << std::endl;
		return 0;
	}

	double A_EARTH = 6378137.0;
	double flattening = 1.0/298.257223563;
	double NAV_E2 = (2.0-flattening)*flattening; // also e^2
	double deg2rad = M_PI/180.0;

	double slat = sin(lla[0]*deg2rad);
	double clat = cos(lla[0]*deg2rad);
	double r_n = A_EARTH/sqrt(1.0 - NAV_E2*slat*slat);
	xyz[0] = (r_n + lla[2])*clat*cos(lla[1]*deg2rad);  
	xyz[1] = (r_n + lla[2])*clat*sin(lla[1]*deg2rad);  
	xyz[2] = (r_n*(1.0 - NAV_E2) + lla[2])*slat;

  	return 1;
}

//------------------------------------------------------------------------------------------------
// WgsConversions::ned2xyz [Public]  --- convert from (North,East,Down) to (ECEF X, ECEF Y, ECEF Z)
//------------------------------------------------------------------------------------------------
bool WgsConversions::ned2xyz(double xyz[3], const double ned[3], const double ref_lla[3]){

    double enu[3]; 

    // Convert NED to ENU
    enu_nedFlip(enu,ned);

    if(!enu2xyz(xyz,enu,ref_lla))
        return 0;

    return 1; 
}

bool WgsConversions::ned2xyz(double xyz[3], const double ned[3]){

	if(!reference_lla_flag)
		std::cout << "Attempted ned2xyz..." << std::endl;
	    std::cout << "No reference LLA setup or inputted" << std::endl;
	    std::cout << "ned2xyz failed" <<std::endl; 
		return 0; 

	if(!ned2xyz(xyz,ned,reference_lla))
		return 0; 

	return 1; 
}

//------------------------------------------------------------------------------------------------
// WgsConversions::xyz2ned [Public]  --- convert from (ECEF X, ECEF Y, ECEF Z) to (North,East,Down)
//------------------------------------------------------------------------------------------------
bool WgsConversions::xyz2ned(double ned[3], const double xyz[3], const double ref_lla[3]){
    
    double enu[3];

    if(!xyz2enu(enu,xyz,ref_lla))
        return 0; 

    // Convert from ENU to NED 
    enu_nedFlip(ned,enu);

    return 1; 
}

bool WgsConversions::xyz2ned(double ned[3], const double xyz[3]){

	if(!reference_lla_flag)
		std::cout << "Attempted xyz2ned..." << std::endl;
	    std::cout << "No reference LLA setup or inputted" << std::endl;
	    std::cout << "xyz2ned failed" <<std::endl; 
		return 0; 

	if(!xyz2ned(ned,xyz,reference_lla))
		return 0; 

	return 1; 
}

//------------------------------------------------------------------------------------------------
// WgsConversions::enu2xyz [Public]  --- convert from (East,North,Up) to (ECEF X, ECEF Y, ECEF Z)
//------------------------------------------------------------------------------------------------
bool WgsConversions::enu2xyz(double xyz[3], const double enu[3], const double ref_lla[3]){

  double lla[3];

  // first enu2lla
  if(!enu2lla(lla,enu, ref_lla))
  	return 0;

  // then lla2xyz
  if(!lla2xyz(xyz,lla))
  	return 0;

  return 1;
}

bool WgsConversions::enu2xyz(double xyz[3], const double enu[3]){

	if(!reference_lla_flag)
		std::cout << "Attempted enu2xyz..." << std::endl;
	    std::cout << "No reference LLA setup or inputted" << std::endl;
	    std::cout << "enu2xyz failed" <<std::endl; 
		return 0; 

	if(!enu2xyz(xyz,enu,reference_lla))
		return 0; 

	return 1; 
}

//------------------------------------------------------------------------------------------------
// WgsConversions::xyz2enu [Public]  --- convert from (ECEF X, ECEF Y, ECEF Z) to (East,North,Up)
//------------------------------------------------------------------------------------------------
bool WgsConversions::xyz2enu(double enu[3], const double xyz[3], const double ref_lla[3]){
  
  	double ref_xyz[3],diff_xyz[3],R[3][3];

	// First, calculate the xyz of reflat, reflon, refalt
    if (!lla2xyz(ref_xyz,ref_lla))
		return 0;
	
    //Difference xyz from reference point
    diff_xyz[0] = xyz[0] - ref_xyz[0];
    diff_xyz[1] = xyz[1] - ref_xyz[1];
    diff_xyz[2] = xyz[2] - ref_xyz[2];

    rot3d(R, ref_lla[0], ref_lla[1]);

    matrixMultiply(enu,R,diff_xyz);

    return 1;
}

bool WgsConversions::xyz2enu(double enu[3], const double xyz[3]){

	if(!reference_lla_flag)
		std::cout << "Attempted xyz2enu..." << std::endl;
	    std::cout << "No reference LLA setup or inputted" << std::endl;
	    std::cout << "xyz2enu failed" <<std::endl; 
		return 0; 

	if(!xyz2enu(enu,xyz,reference_lla))
		return 0; 

	return 1; 
}

//--------------------------------------------------------------------------------------------------------------
// WgsConversions::xyz2ned_vel [Public]  --- convert velocities from (ECEF X, ECEF Y, ECEF Z) to (North,East,Down)
//--------------------------------------------------------------------------------------------------------------
void WgsConversions::xyz2ned_vel(double ned_vel[3], const double xyz_vel[3], const double ref_lla[3]){

    double enu_vel[3];

    xyz2enu_vel(enu_vel,xyz_vel,ref_lla); 

    enu_nedFlip(ned_vel,enu_vel);
}

void WgsConversions::xyz2ned_vel(double ned_vel[3], const double xyz_vel[3]){

	if(!reference_lla_flag)
		std::cout << "Attempted xyz2ned_vel..." << std::endl;
	    std::cout << "No reference LLA setup or inputted" << std::endl;
	    std::cout << "xyz2ned_vel failed" <<std::endl; 

    xyz2ned_vel(ned_vel,xyz_vel,reference_lla);
}

//--------------------------------------------------------------------------------------------------------------
// WgsConversions::ned2xyz_vel [Public]  --- convert velocities from (North,East,Down) to (ECEF X, ECEF Y, ECEF Z)
//--------------------------------------------------------------------------------------------------------------
void WgsConversions::ned2xyz_vel(double xyz_vel[3], const double ned_vel[3], const double ref_lla[3]){

    double enu_vel[3]; 

    enu_nedFlip(enu_vel,ned_vel);

    enu2xyz_vel(xyz_vel,enu_vel,ref_lla);      
}

void WgsConversions::ned2xyz_vel(double xyz_vel[3], const double ned_vel[3]){

	if(!reference_lla_flag)
		std::cout << "Attempted ned2xyz_vel..." << std::endl;
	    std::cout << "No reference LLA setup or inputted" << std::endl;
	    std::cout << "ned2xyz_vel failed" <<std::endl; 

    ned2xyz_vel(xyz_vel,ned_vel,reference_lla);
}

//--------------------------------------------------------------------------------------------------------------
// WgsConversions::xyz2enu_vel [Public]  --- convert velocities from (ECEF X, ECEF Y, ECEF Z) to (East,North,Up)
//--------------------------------------------------------------------------------------------------------------
void WgsConversions::xyz2enu_vel(double enu_vel[3], const double xyz_vel[3], const double ref_lla[3]){
  
    double R[3][3];

    rot3d(R, ref_lla[0], ref_lla[1]);

    matrixMultiply(enu_vel,R,xyz_vel);

}

void WgsConversions::xyz2enu_vel(double enu_vel[3], const double xyz_vel[3]){

	if(!reference_lla_flag)
		std::cout << "Attempted xyz2enu_vel..." << std::endl;
	    std::cout << "No reference LLA setup or inputted" << std::endl;
	    std::cout << "xyz2enu_vel failed" <<std::endl; 

    xyz2enu_vel(enu_vel,xyz_vel,reference_lla);
}

//--------------------------------------------------------------------------------------------------------------
// WgsConversions::enu2xyz_vel [Public]  --- convert velocities from (East,North,Up) to (ECEF X, ECEF Y, ECEF Z)
//--------------------------------------------------------------------------------------------------------------
void WgsConversions::enu2xyz_vel(double xyz_vel[3], const double enu_vel[3], const double ref_lla[3]){
    
    double R[3][3],Rt[3][3];

    rot3d(R, ref_lla[0], ref_lla[1]);

    transposeMatrix(Rt,R);

    matrixMultiply(xyz_vel,Rt,enu_vel);

}

void WgsConversions::enu2xyz_vel(double xyz_vel[3], const double enu_vel[3]){

	if(!reference_lla_flag)
		std::cout << "Attempted enu2xyz_vel..." << std::endl;
	    std::cout << "No reference LLA setup or inputted" << std::endl;
	    std::cout << "enu2xyz_vel failed" <<std::endl; 

    enu2xyz_vel(xyz_vel,enu_vel,reference_lla);
}

//--------------------------------------------------------------------------------------------------------------------------------
// WgsConversions::xyz2ned_cov [Public]  --- convert position/velocity covariance from (ECEF X, ECEF Y, ECEF Z) to (North,East,Down)
//--------------------------------------------------------------------------------------------------------------------------------
void WgsConversions::xyz2ned_cov(double ned_Cov[3][3], const double xyz_Cov[3][3], const double ref_lla[3]){
  
    double R[3][3],R2[3][3],Rflip[3][3],Rt[3][3],Tmp[3][3];

    rot3d(R, ref_lla[0], ref_lla[1]);

    enu_nedFlip(Rflip);

    enu_nedFlip(Rflip);

    matrixMultiply(R2,Rflip,R);

    transposeMatrix(Rt,R2);

    matrixMultiply(Tmp,xyz_Cov,Rt); // Tmp = xyz_cov*R'

    matrixMultiply(ned_Cov,R2,Tmp); // enu_cov = R*xyz_cov*R'
}

void WgsConversions::xyz2ned_cov(double ned_Cov[3][3], const double xyz_Cov[3][3]){

	if(!reference_lla_flag)
		std::cout << "Attempted xyz2ned_cov..." << std::endl;
	    std::cout << "No reference LLA setup or inputted" << std::endl;
	    std::cout << "xyz2ned_cov failed" <<std::endl; 

    xyz2ned_cov(ned_Cov,xyz_Cov,reference_lla);
}

void WgsConversions::xyz2ned_cov(double ned_cov[9], const double xyz_cov[9], const double ref_lla[3]){
  
    double xyz_Cov[3][3],ned_Cov[3][3];

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            xyz_Cov[i][j]=xyz_cov[3*i+j];

    double R[3][3],R2[3][3],Rflip[3][3],Rt[3][3],Tmp[3][3];

    rot3d(R, ref_lla[0], ref_lla[1]);

    enu_nedFlip(Rflip);

    matrixMultiply(R2,Rflip,R);

    transposeMatrix(Rt,R2);

    matrixMultiply(Tmp,xyz_Cov,Rt);

    matrixMultiply(ned_Cov,R2,Tmp);

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            ned_cov[3*i+j] = ned_Cov[i][j];

}

void WgsConversions::xyz2ned_cov(double ned_Cov[9], const double xyz_Cov[9]){

	if(!reference_lla_flag)
		std::cout << "Attempted xyz2ned_cov..." << std::endl;
	    std::cout << "No reference LLA setup or inputted" << std::endl;
	    std::cout << "xyz2ned_cov failed" <<std::endl; 

    xyz2ned_cov(ned_Cov,xyz_Cov,reference_lla);
}

//--------------------------------------------------------------------------------------------------------------------------------
// WgsConversions::xyz2enu_cov [Public]  --- convert position/velocity covariance from (ECEF X, ECEF Y, ECEF Z) to (East,North,Up)
//--------------------------------------------------------------------------------------------------------------------------------
void WgsConversions::xyz2enu_cov(double enu_Cov[3][3], const double xyz_Cov[3][3], const double ref_lla[3]){
  
    double R[3][3],Rt[3][3],Tmp[3][3];

    rot3d(R, ref_lla[0], ref_lla[1]);

    transposeMatrix(Rt,R);

    matrixMultiply(Tmp,xyz_Cov,Rt); // Tmp = xyz_cov*R'

    matrixMultiply(enu_Cov,R,Tmp); // enu_cov = R*xyz_cov*R'
}

void WgsConversions::xyz2enu_cov(double enu_Cov[3][3], const double xyz_Cov[3][3]){

	if(!reference_lla_flag)
		std::cout << "Attempted xyz2enu_cov..." << std::endl;
	    std::cout << "No reference LLA setup or inputted" << std::endl;
	    std::cout << "xyz2enu_cov failed" <<std::endl; 

    xyz2enu_cov(enu_Cov,xyz_Cov,reference_lla);
}

void WgsConversions::xyz2enu_cov(double enu_cov[9], const double xyz_cov[9], const double ref_lla[3]){
  
    double xyz_Cov[3][3],enu_Cov[3][3];

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            xyz_Cov[i][j]=xyz_cov[3*i+j];

    double R[3][3],Rt[3][3],Tmp[3][3];

    rot3d(R, ref_lla[0], ref_lla[1]);

    transposeMatrix(Rt,R);

    matrixMultiply(Tmp,xyz_Cov,Rt);

    matrixMultiply(enu_Cov,R,Tmp);

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            enu_cov[3*i+j] = enu_Cov[i][j];

}

void WgsConversions::xyz2enu_cov(double enu_Cov[9], const double xyz_Cov[9]){

	if(!reference_lla_flag)
		std::cout << "Attempted xyz2enu_cov..." << std::endl;
	    std::cout << "No reference LLA setup or inputted" << std::endl;
	    std::cout << "xyz2enu_cov failed" <<std::endl; 

    xyz2ned_cov(enu_Cov,xyz_Cov,reference_lla);
}


//--------------------------------------------------------------------------------------------------------------------------------
// WgsConversions::ned2xyz_cov [Public]  --- convert position/velocity covariance from (North,East,Down) to (ECEF X, ECEF Y, ECEF Z)
//--------------------------------------------------------------------------------------------------------------------------------
void WgsConversions::ned2xyz_cov(double xyz_Cov[3][3], const double ned_Cov[3][3], const double ref_lla[3]){
    
    double R[3][3],R2[3][3],R2t[3][3],Rflip[3][3],Rt[3][3],Tmp[3][3];

    rot3d(R, ref_lla[0], ref_lla[1]);

    transposeMatrix(Rt,R);

    enu_nedFlip(Rflip) ; 

    matrixMultiply(R2,Rt,Rflip);

    matrixMultiply(Tmp,R2,ned_Cov);

    transposeMatrix(R2t,R2); 

    matrixMultiply(xyz_Cov,Tmp,R2t);
}

void WgsConversions::ned2xyz_cov(double xyz_Cov[3][3], const double ned_Cov[3][3]){

	if(!reference_lla_flag)
		std::cout << "Attempted ned2xyz_cov..." << std::endl;
	    std::cout << "No reference LLA setup or inputted" << std::endl;
	    std::cout << "ned2xyz_cov failed" <<std::endl; 

    ned2xyz_cov(xyz_Cov,ned_Cov,reference_lla);
}

void WgsConversions::ned2xyz_cov(double xyz_cov[9], const double ned_cov[9], const double ref_lla[3]){
    
    double xyz_Cov[3][3],ned_Cov[3][3];

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            ned_Cov[i][j]=ned_cov[3*i+j];

    double R[3][3],R2[3][3],Rflip[3][3],Rt[3][3],R2t[3][3],Tmp[3][3];

    rot3d(R, ref_lla[0], ref_lla[1]);

    transposeMatrix(Rt,R);

    enu_nedFlip(Rflip) ; 

    matrixMultiply(R2,Rt,Rflip);

    matrixMultiply(Tmp,R2,ned_Cov);

    transposeMatrix(R2t,R2); 

    matrixMultiply(xyz_Cov,Tmp,R2t);

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            xyz_cov[3*i+j] = xyz_Cov[i][j];
}

void WgsConversions::ned2xyz_cov(double xyz_Cov[9], const double ned_Cov[9]){

	if(!reference_lla_flag)
		std::cout << "Attempted ned2xyz_cov..." << std::endl;
	    std::cout << "No reference LLA setup or inputted" << std::endl;
	    std::cout << "ned2xyz_cov failed" <<std::endl; 

    ned2xyz_cov(xyz_Cov,ned_Cov,reference_lla);
}

//--------------------------------------------------------------------------------------------------------------------------------
// WgsConversions::enu2xyz_cov [Public]  --- convert position/velocity covariance from (East,North,Up) to (ECEF X, ECEF Y, ECEF Z)
//--------------------------------------------------------------------------------------------------------------------------------
void WgsConversions::enu2xyz_cov(double xyz_Cov[3][3], const double enu_Cov[3][3], const double ref_lla[3]){
    
    double R[3][3],Rt[3][3],Tmp[3][3];

    rot3d(R, ref_lla[0], ref_lla[1]);

    transposeMatrix(Rt,R);

    matrixMultiply(Tmp,enu_Cov,R);

    matrixMultiply(xyz_Cov,Rt,Tmp);

}

void WgsConversions::enu2xyz_cov(double xyz_Cov[3][3], const double enu_Cov[3][3]){

	if(!reference_lla_flag)
		std::cout << "Attempted enu2xyz_cov..." << std::endl;
	    std::cout << "No reference LLA setup or inputted" << std::endl;
	    std::cout << "enu2xyz_cov failed" <<std::endl; 

    enu2xyz_cov(xyz_Cov,enu_Cov,reference_lla);
}

void WgsConversions::enu2xyz_cov(double xyz_cov[9], const double enu_cov[9], const double ref_lla[3]){
    
    double xyz_Cov[3][3],enu_Cov[3][3];

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            enu_Cov[i][j]=enu_cov[3*i+j];

    double R[3][3],Rt[3][3],Tmp[3][3];

    rot3d(R, ref_lla[0], ref_lla[1]);

    transposeMatrix(Rt,R);

    matrixMultiply(Tmp,enu_Cov,R); 

    matrixMultiply(xyz_Cov,Rt,Tmp);

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            xyz_cov[3*i+j] = xyz_Cov[i][j];
}

void WgsConversions::enu2xyz_cov(double xyz_Cov[9], const double enu_Cov[9]){

	if(!reference_lla_flag)
		std::cout << "Attempted enu2xyz_cov..." << std::endl;
	    std::cout << "No reference LLA setup or inputted" << std::endl;
	    std::cout << "enu2xyz_cov failed" <<std::endl; 

    enu2xyz_cov(xyz_Cov,enu_Cov,reference_lla);
}

//--------------------------------------------------------------------------------------------
// WgsConversions::rotation2euler [Public]  ---  return the Euler Angles
//--------------------------------------------------------------------------------------------
void WgsConversions::rotation2euler(double ang[3], const double R[3][3],bool body2nav_or_nav2body){

    // If body2nav_or_nav2body (1-->body2nav,0-->nav2body)
    if(!body2nav_or_nav2body){
        ang[0] = atan2(R[2][1],R[2][2]);
        ang[1] = -asin(R[2][0]);
        ang[2] = atan2(R[1][0],R[0][0]);
    }
    if(body2nav_or_nav2body){

        ang[0] = atan2(R[1][2],R[2][2]);
        ang[1] = -asin(R[0][2]);
        ang[2] = atan2(R[0][1],R[0][0]);
    }
}

//--------------------------------------------------------------------------------------------
// WgsConversions::nav2bodyRotation [Public]  --- return the Rotation Matrix Values
//--------------------------------------------------------------------------------------------
void WgsConversions::nav2bodyRotation(double R[3][3], const double ang[3]){
    double Rx[3][3],Ry[3][3],Rz[3][3],R2[3][3]; 

    rot(Rx,ang[0],1);
    rot(Ry,ang[1],2);
    rot(Rz,ang[2],3); 

    matrixMultiply(R2,Ry,Rx);
    matrixMultiply(R,Rz,R2);
}

//--------------------------------------------------------------------------------------------
// WgsConversions::body2navRotation [Public] --- return the Rotation Matrix Values
//--------------------------------------------------------------------------------------------
void WgsConversions::body2navRotation(double R[3][3], const double ang[3]){
    double R1[3][3];
    nav2bodyRotation(R1,ang);
    transposeMatrix(R,R1);
}

//--------------------------------------------------------------------------------------------
// WgsConversions::nav2xyzRotation [Public]  --- return the Rotation Matrix Values
//--------------------------------------------------------------------------------------------
void WgsConversions::nav2xyzRotation(double R[3][3], const double ref_lla[3]){

    double clat = cos(ref_lla[0] * M_PI / 180) ;
    double slat = sin(ref_lla[0] * M_PI / 180) ;
    double clon = cos(ref_lla[1] * M_PI / 180) ;
    double slon = sin(ref_lla[1] * M_PI / 180) ; 

    R[0][0] = -slat*clon ; 
    R[0][1] = -slon ;
    R[0][2] = -clat*clon ;
    R[1][0] = -slat*slon ;
    R[1][1] = clon ;
    R[1][2] = -clat*slon ;
    R[2][0] = clat ;
    R[2][1] = 0 ;
    R[2][2] = -slat ;
}

void WgsConversions::nav2xyzRotation(double R[3][3]){

    nav2xyzRotation(R,reference_lla); 
}

//--------------------------------------------------------------------------------------------
// WgsConversions::xyz2navRotation [Public]  --- return the Rotation Matrix Values
//--------------------------------------------------------------------------------------------
void WgsConversions::xyz2navRotation(double R[3][3], const double ref_lla[3]){

    double clat = cos(ref_lla[0] * M_PI / 180) ;
    double slat = sin(ref_lla[0] * M_PI / 180) ;
    double clon = cos(ref_lla[1] * M_PI / 180) ;
    double slon = sin(ref_lla[1] * M_PI / 180) ; 

    R[0][0] = -slat*clon ; 
    R[0][1] = -slat*slon ;
    R[0][2] = clat ;
    R[1][0] = -slon ;
    R[1][1] = clon ;
    R[1][2] = 0 ;
    R[2][0] = -clat*clon ;
    R[2][1] = -clat*slon ;
    R[2][2] = -slat ;
}

void WgsConversions::xyz2navRotation(double R[3][3]){

    nav2xyzRotation(R,reference_lla); 
}
 
// -------------------------------------------------------------------------------------------
// WgsConversions::body2xyzRotation [Public]  --- return the Rotation Matrix Values
// -------------------------------------------------------------------------------------------
void WgsConversions::body2xyzRotation(double R[3][3], const double ang[3], const double ref_lla[3]){
    double R1[3][3],R2[3][3];
    body2navRotation(R1,ang);
    nav2xyzRotation(R2,ref_lla); 
    matrixMultiply(R,R2,R1);
}

void WgsConversions::body2xyzRotation(double R[3][3], const double ang[3]){
    body2xyzRotation(R,ang,reference_lla);
}

// -------------------------------------------------------------------------------------------
// WgsConversions::xyz2bodyRotation [Public]  --- return the Rotation Matrix Values
// -------------------------------------------------------------------------------------------
void WgsConversions::xyz2bodyRotation(double R[3][3], const double ang[3], const double ref_lla[3]){
    double R1[3][3],R2[3][3];
    nav2bodyRotation(R1,ang);
    xyz2navRotation(R2,ref_lla); 
    matrixMultiply(R,R1,R2);
}

void WgsConversions::xyz2bodyRotation(double R[3][3], const double ang[3]){
    xyz2bodyRotation(R,ang,reference_lla);
}


//--------------------------------------------------------------------------------------------
// WgsConversions::enu_nedFlip [Private]  --- return the switch from ENU to NED or Vice Versa
//--------------------------------------------------------------------------------------------
void WgsConversions::enu_nedFlip(double flip[3], const double enu_or_ned[3]){

    double R[3][3];
    R[0][0] = 0;
    R[0][1] = 1;
    R[0][2] = 0;
    R[1][0] = 1;
    R[1][1] = 0;
    R[1][2] = 0;
    R[2][0] = 0;
    R[2][1] = 0;
    R[2][2] = -1;

    matrixMultiply(flip,R,enu_or_ned);
}

//--------------------------------------------------------------------------------------------
// WgsConversions::enu_nedFlip [Private]  --- return the switch from ENU to NED or Vice Versa
//--------------------------------------------------------------------------------------------
void WgsConversions::enu_nedFlip(double R[3][3]){

    R[0][0] = 0;
    R[0][1] = 1;
    R[0][2] = 0;
    R[1][0] = 1;
    R[1][1] = 0;
    R[1][2] = 0;
    R[2][0] = 0;
    R[2][1] = 0;
    R[2][2] = -1;
}

//--------------------------------------------------------------------------------------------
// WgsConversions::enu2xyz [Private]  --- return the 3D rotation matrix to/from ECEF/ENU frame
//--------------------------------------------------------------------------------------------
void WgsConversions::rot3d(double R[3][3], const double reflat, const double reflon){

    double R1[3][3],R2[3][3];

    rot(R1, 90 + reflon, 3);
    rot(R2, 90 - reflat, 1);

    matrixMultiply(R, R2, R1);
}

//------------------------------------------------------------------------------------------------
// WgsConversions::matrixMultiply [Private]  --- Multiply 3x3 matrix times another 3x3 matrix C=AB
//------------------------------------------------------------------------------------------------
void WgsConversions::matrixMultiply(double C[3][3], const double A[3][3], const double B[3][3]){

    C[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0];
    C[0][1] = A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1];
    C[0][2] = A[0][0] * B[0][2] + A[0][1] * B[1][2] + A[0][2] * B[2][2];
    C[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0];
    C[1][1] = A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1];
    C[1][2] = A[1][0] * B[0][2] + A[1][1] * B[1][2] + A[1][2] * B[2][2];
    C[2][0] = A[2][0] * B[0][0] + A[2][1] * B[1][0] + A[2][2] * B[2][0];
    C[2][1] = A[2][0] * B[0][1] + A[2][1] * B[1][1] + A[2][2] * B[2][1];
    C[2][2] = A[2][0] * B[0][2] + A[2][1] * B[1][2] + A[2][2] * B[2][2];

}

//------------------------------------------------------------------------------------------------
// WgsConversions::matrixMultiply [Private]  --- Multiply 3x3 matrix times a 3x1 vector c=Ab
//------------------------------------------------------------------------------------------------
void WgsConversions::matrixMultiply(double c[3], const double A[3][3], const double b[3]){

    c[0] = A[0][0] * b[0] + A[0][1] * b[1] + A[0][2] * b[2];
    c[1] = A[1][0] * b[0] + A[1][1] * b[1] + A[1][2] * b[2];
    c[2] = A[2][0] * b[0] + A[2][1] * b[1] + A[2][2] * b[2];

}

//------------------------------------------------------------------------------------------------
// WgsConversions::transposeMatrix [Private]  --- transpose a 3x3 matrix At = A'
//------------------------------------------------------------------------------------------------
void WgsConversions::transposeMatrix(double At[3][3], const double A[3][3]){

    At[0][0] = A[0][0];
    At[0][1] = A[1][0];
    At[0][2] = A[2][0];
    At[1][0] = A[0][1];
    At[1][1] = A[1][1];
    At[1][2] = A[2][1];
    At[2][0] = A[0][2];
    At[2][1] = A[1][2];
    At[2][2] = A[2][2];

}

//------------------------------------------------------------------------------------------------
// WgsConversions::rot [Private]  --- rotation matrix
//------------------------------------------------------------------------------------------------
void WgsConversions::rot(double R[3][3], const double angle, const int axis) {

    double cang = cos(angle * M_PI / 180);
    double sang = sin(angle * M_PI / 180);

    if (axis == 1) {
        R[0][0] = 1;
        R[0][1] = 0;
        R[0][2] = 0;
        R[1][0] = 0;
        R[2][0] = 0;
        R[1][1] = cang;
        R[2][2] = cang;
        R[1][2] = sang;
        R[2][1] = -sang;
    } else if (axis == 2) {
        R[0][1] = 0;
        R[1][0] = 0;
        R[1][1] = 1;
        R[1][2] = 0;
        R[2][1] = 0;
        R[0][0] = cang;
        R[2][2] = cang;
        R[0][2] = -sang;
        R[2][0] = sang;
    } else if (axis == 3) {
        R[2][0] = 0;
        R[2][1] = 0;
        R[2][2] = 1;
        R[0][2] = 0;
        R[1][2] = 0;
        R[0][0] = cang;
        R[1][1] = cang;
        R[1][0] = -sang;
        R[0][1] = sang;
    }
}