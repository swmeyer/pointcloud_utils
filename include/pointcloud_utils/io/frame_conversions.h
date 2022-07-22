#ifndef FRAME_CONVERSIONS_H
#define FRAME_CONVERSIONS_H

/*
 * Header file for wgs_conversions.cpp
 *
 * Dan Pierce
 * 2017-03-13
 */
#include <iostream>
#include <math.h>

typedef double array_type[3];
typedef double matrix_type[3][3];

/*! Primary class for wgs_conversions */
class WgsConversions{

  public:
    WgsConversions();
    ~WgsConversions();

    /* Refence LLA */
    array_type reference_lla ;
    bool reference_lla_flag; 

    /*! Update Reference LLA */ 
    void updateReferenceLLA(double ref_lla[3],bool rad_or_deg);
    void updateReferenceLLA(double ref_lla[3]); 

    /*! Output Rotation Matrix Body2Nav */
    void body2navRotation(double R[3][3], const double ang[3]);
    void nav2bodyRotation(double R[3][3], const double ang[3]);

    /*! Output Rotation Matrix Nav2XYZ */ 
    void nav2xyzRotation(double R[3][3], const double ref_lla[3]); 
    void nav2xyzRotation(double R[3][3]);
    void xyz2navRotation(double R[3][3], const double ref_lla[3]);
    void xyz2navRotation(double R[3][3]);

    /*! Output Rotation Matrix body2XYZ */
    void body2xyzRotation(double R[3][3], const double ang[3], const double ref_lla[3]); 
    void body2xyzRotation(double R[3][3], const double ang[3]);
    void xyz2bodyRotation(double R[3][3], const double ang[3], const double ref_lla[3]);
    void xyz2bodyRotation(double R[3][3], const double ang[3]);

    /* Output Euler from Rotation Matrix */
    void rotation2euler(double ang[3],const double R[3][3], bool body2nav_or_nav2body); 

    /*! Convert to/from ENU/NED */
    bool ned2enu(double enu[3], const double ned[3]);
    bool enu2ned(double ned[3], const double enu[3]);

    /*! Convert to/from NED/LLA (requires reference LLA) */
    bool ned2lla(double lla[3], const double ned[3], const double ref_lla[3]);
    bool lla2ned(double ned[3], const double lla[3], const double ref_lla[3]);
    bool ned2lla(double lla[3], const double ned[3]);
    bool lla2ned(double ned[3], const double lla[3]);

    /*! Convert to/from ENU/LLA (requires reference LLA) */
    bool enu2lla(double lla[3], const double enu[3], const double ref_lla[3]);
    bool lla2enu(double enu[3], const double lla[3], const double ref_lla[3]);
    bool enu2lla(double lla[3], const double enu[3]);
    bool lla2enu(double enu[3], const double lla[3]);

    /*! Convert to/from ECEF/LLA */
    bool xyz2lla(double lla[3], const double xyz[3]);
    bool lla2xyz(double xyz[3], const double lla[3]);

    /*! Convert to/from ENU/ECEF (requires reference LLA) */
    bool ned2xyz(double xyz[3], const double ned[3], const double ref_lla[3]);
    bool xyz2ned(double ned[3], const double xyz[3], const double ref_lla[3]);
    bool ned2xyz(double xyz[3], const double ned[3]);
    bool xyz2ned(double ned[3], const double xyz[3]);

    /*! Convert to/from ENU/ECEF (requires reference LLA) */
    bool enu2xyz(double xyz[3], const double enu[3], const double ref_lla[3]);
    bool xyz2enu(double enu[3], const double xyz[3], const double ref_lla[3]);
    bool enu2xyz(double xyz[3], const double enu[3]);
    bool xyz2enu(double enu[3], const double xyz[3]);

    /*! Convert velocities (or delta positions) to/from NED/ECEF (requires reference LLA) */
    void ned2xyz_vel(double xyz_vel[3], const double ned_vel[3], const double ref_lla[3]);
    void xyz2ned_vel(double ned_vel[3], const double xyz_vel[3], const double ref_lla[3]);
    void ned2xyz_vel(double xyz_vel[3], const double ned_vel[3]);
    void xyz2ned_vel(double ned_vel[3], const double xyz_vel[3]);

    /*! Convert velocities (or delta positions) to/from ENU/ECEF (requires reference LLA) */
    void enu2xyz_vel(double xyz_vel[3], const double enu_vel[3], const double ref_lla[3]);
    void xyz2enu_vel(double enu_vel[3], const double xyz_vel[3], const double ref_lla[3]);
    void enu2xyz_vel(double xyz_vel[3], const double enu_vel[3]);
    void xyz2enu_vel(double enu_vel[3], const double xyz_vel[3]);

    /*! Convert position/velocity covariance to/from ENU/ECEF (requires reference LLA) */
    void enu2xyz_cov(double xyz_Cov[3][3], const double enu_Cov[3][3], const double ref_lla[3]);
    void xyz2enu_cov(double enu_Cov[3][3], const double xyz_Cov[3][3], const double ref_lla[3]);
    void enu2xyz_cov(double xyz_Cov[3][3], const double enu_Cov[3][3]);
    void xyz2enu_cov(double enu_Cov[3][3], const double xyz_Cov[3][3]);

    void enu2xyz_cov(double xyz_cov[9], const double enu_cov[9], const double ref_lla[3]);
    void xyz2enu_cov(double enu_cov[9], const double xyz_cov[9], const double ref_lla[3]);
    void enu2xyz_cov(double xyz_cov[9], const double enu_cov[9]);
    void xyz2enu_cov(double enu_cov[9], const double xyz_cov[9]);

    /*! Convert position/velocity covariance to/from NED/ECEF (requires reference LLA) */
    void ned2xyz_cov(double xyz_Cov[3][3], const double ned_Cov[3][3], const double ref_lla[3]);
    void xyz2ned_cov(double ned_Cov[3][3], const double xyz_Cov[3][3], const double ref_lla[3]);
    void ned2xyz_cov(double xyz_Cov[3][3], const double ned_Cov[3][3]);
    void xyz2ned_cov(double ned_Cov[3][3], const double xyz_Cov[3][3]);
    
    void ned2xyz_cov(double xyz_cov[9], const double ned_cov[9], const double ref_lla[3]);
    void xyz2ned_cov(double ned_cov[9], const double xyz_cov[9], const double ref_lla[3]);
    void ned2xyz_cov(double xyz_cov[9], const double ned_cov[9]);
    void xyz2ned_cov(double ned_cov[9], const double xyz_cov[9]);

  private:

    /*! Rotation matrix about a given axis */
    void rot(double R[3][3], const double angle, const int axis);
    
    /*! Rotation matrix from ECEF to ENU frame */
    void rot3d(double R[3][3], const double reflat, const double reflon);
    
    /*! Multiply 3x3 matrix times another 3x3 matrix C=AB */
    void matrixMultiply(double C[3][3], const double A[3][3], const double B[3][3]);
    
    /*! Multiply 3x3 matrix times a 3x1 vector c=Ab */
    void matrixMultiply(double c[3], const double A[3][3], const double b[3]);

    /*! ENU/NED Flip */
    void enu_nedFlip(double flip[3], const double enu_or_ned[3]);
    void enu_nedFlip(double R[3][3]);
    
    /*! Transpose a 3x3 matrix At = A' */
    void transposeMatrix(double At[3][3], const double A[3][3]);
    
};


#endif