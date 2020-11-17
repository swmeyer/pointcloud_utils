/** Author: Stephanie Meyer swmeyer16@gmail.com 12 Nov 2020
 * Brief: Transformer class for point clouds - can handle many different rotation schema
 * File: pointcloud_transformer.cpp
 */

// -------------------------------
#include "pointcloud_utils/conversion/pointcloud_transformer.hpp"
// -------------------------------

namespace pointcloud_utils
{

	PointCloudTransformer::PointCloudTransformer()
	{

	}
	
	PointCloudTransformer::~PointCloudTransformer()
	{

	}


	/**
	 * @function 	transformCloud
	 * @brief 		transforms the given cloud by the given transform (assumes either body or world ordered rotations)
	 * @param 		cloud - points to transform
	 * @param 		transform - holder of the rotation and translation values to use
	 * @param 		angle_order - to specify the order the given rotations should be applied in (RPY, YPR, RPR, etc)
	 * @param 		rotation_frame - specifies whether to expect the inputed rotation angles to be used with respect to a fixed world coordinate or
	 * 							the local cloud coordinate
	 * @param 		fwd_transform - if true, will apply the transform as-is. If false, will apply the inverse transform
	 * @param 		translate_first - if true, apply the translation in the original frame, otherwise, apply after rotation
	 * @param 		publish_transform - if true, broadcast a tf to the tf tree. if false, don't publish transform
	 * @return 		void
	 */
	void PointCloudTransformer::transformCloud
	(
		std::vector<pointstruct>& cloud,
		const Transform& transform, 
		const PointCloudTransformer::angleOrder& angle_order, 
		const PointCloudTransformer::rotationFrame& rotation_frame,
		const bool& fwd_transform,
		const bool& translate_first,
		const bool& publish_transform
	)
	{
		std::cout << "Transforming cloud with angle order " << angle_order << "\n";
		//Generate transform matrix:
		Eigen::MatrixXf transform_matrix;
		generateTransformMatrix(transform_matrix, transform, angle_order, rotation_frame, translate_first);

		std::cout << "generated transform: \n" << transform_matrix << "\n";

		//Generate point matrix:
		Eigen::MatrixXf point_matrix(4, cloud.size());

		int i = 0;
		for (pointstruct pt : cloud)
		{
			point_matrix.col(i) << pt.x, pt.y, pt.z, 1;
			i++;
		}

		//Check transform direction:
		if (!fwd_transform)
		{
			transform_matrix = transform_matrix.inverse();
		}

		//Apply transform:
		Eigen::MatrixXf transformed_matrix(4, cloud.size());
		transformed_matrix = transform_matrix * point_matrix;

		//Save transformed points into the cloud:
		for (uint i = 0; i < cloud.size(); i++)
		{
			 cloud[i].x = transformed_matrix.col(i)[0];
			 cloud[i].y = transformed_matrix.col(i)[1];
			 cloud[i].z = transformed_matrix.col(i)[2];
		}

		//Publish transform
		if (publish_transform)
		{
			//TODO: broadcast tf
			std::cout << "Warning: tf publish function is not implemented\n";
		}
	}

	/**
	 * @function 	getAngleOrderType
	 * @brief 		given a string of 3 letters representing the angle order,
	 *				determine which enumerated angle order is being requested
	 * @param 		str - the string containing the angle order request
	 * @return 		angleOrder - the enumerated value of the given angle order, 
	 *				or -1 if request was invalid
	 */
	PointCloudTransformer::angleOrder PointCloudTransformer::getAngleOrderType(const std::string& str)
	{
		if (str == "RYR" || str == "ryr")
		{
			return PointCloudTransformer::angleOrder::RYR;
		} else if (str == "RPR" || str == "rpr")
		{
			return PointCloudTransformer::angleOrder::RPR;
		} else if (str == "PRP" || str == "prp")
		{
			return PointCloudTransformer::angleOrder::PRP;
		} else if (str == "PYP" || str == "pyp")
		{
			return PointCloudTransformer::angleOrder::PYP;
		} else if (str == "YPY" || str == "ypy")
		{
			return PointCloudTransformer::angleOrder::YPY;
		} else if (str == "YRY" || str == "YRY")
		{
			return PointCloudTransformer::angleOrder::YRY;
		} else if (str == "RYP" || str == "ryp")
		{
			return PointCloudTransformer::angleOrder::RYP;
		} else if (str == "RPY" || str == "rpy")
		{
			return PointCloudTransformer::angleOrder::RPY;
		} else if (str == "PRY" || str == "pry")
		{
			return PointCloudTransformer::angleOrder::PRY;
		} else if (str == "PYR" || str == "pyr")
		{
			return PointCloudTransformer::angleOrder::PYR;
		} else if (str == "YPR" || str == "ypr")
		{
			return PointCloudTransformer::angleOrder::YPR;
		} else if (str == "YRP" || str == "yrp")
		{
			return PointCloudTransformer::angleOrder::YRP;
		} else
		{
			std::cout << "Warning: Requested invalid angle order: " << str << "\n";
			return PointCloudTransformer::INVALID; //invlaid angle order requested
		}
	}

	/**
	 * @function 	generateTransformMatrix
	 * @brief 		given the transform values and an angle order for the rotations to be applied in (rotation frame either local or world-fixed),
	 * 			    generate a 4x4 affine transform matrix that includes first the rotation by the angles about body-fixed axes and then the translation in
	 * 				the rotated frame
	 * @param 		transform_matrix - place to store the generated transform matrix
	 * @param 		transform - holder of the rotation and translation values to use
	 * @param 		angle_order - the order the given rotations should be applied in
	 * @param 		rotation_frame - the frame the user intended the angles to be applied in
	 * @param 		translate_first - if true, apply the translation in the original frame, otherwise, apply after rotation
	 * @return 		void
	 */
	void PointCloudTransformer::generateTransformMatrix
	(
		Eigen::MatrixXf& transform_matrix, 
		const Transform& transform, 
		const PointCloudTransformer::angleOrder& angle_order, 
		const PointCloudTransformer::rotationFrame& rotation_frame,
		const bool& translate_first
	)
	{
		transform_matrix = Eigen::MatrixXf::Identity(4,4);

		//Useful trig values:
		double cos_roll  = std::cos(transform.roll);
		double cos_pitch = std::cos(transform.pitch);
		double cos_yaw	 = std::cos(transform.yaw);
		double sin_roll  = std::sin(transform.roll);
		double sin_pitch = std::sin(transform.pitch);
		double sin_yaw	 = std::sin(transform.yaw);
		double cos_3 	 = std::cos(transform.last_angle);
		double sin_3 	 = std::sin(transform.last_angle);

		//Rotation Matrix source: https://answers.opencv.org/question/161369/retrieve-yaw-pitch-roll-from-rvec/?answer=162867 
		Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity();

		//TODO: test rotation matricies
		//generate transform matrix
		switch (angle_order)
		{
			case(PointCloudTransformer::angleOrder::RYR):
			{
				std::cout << "Generating RYR transform\n";
				if (rotation_frame == PointCloudTransformer::rotationFrame::WORLD)
				{
					//todo: can I represent a world RPY using body-fixed rotations?
					std::cout << "Warning: world-fixed transforms not yet supported\n";
				} else
				{
					rotation_matrix << 	cos_yaw, 				-cos_3 * sin_yaw, 									sin_yaw * sin_3,
				 				   		cos_roll * sin_yaw, 	(cos_roll * cos_yaw * cos_3 - sin_roll * sin_3), 	(-cos_3 * sin_roll - cos_roll * cos_yaw * sin_3),
				 				   		sin_roll * sin_yaw, 	(cos_roll * sin_3 + cos_yaw * cos_3 * sin_roll), 	(cos_roll * cos_3 - cos_yaw * sin_roll * sin_3);
				}
				break;
			}
			case(PointCloudTransformer::angleOrder::RPR):
			{
				if (rotation_frame == PointCloudTransformer::rotationFrame::WORLD)
				{
					//todo: can I represent a world RPY using body-fixed rotations?
					std::cout << "Warning: world-fixed transforms not yet supported\n";
				} else
				{
					rotation_matrix << 	cos_pitch, 				-sin_pitch * sin_3, 								cos_3 * sin_pitch,
										sin_roll * sin_pitch, 	(cos_roll * cos_3 - cos_pitch * sin_roll * sin_3), 	(-cos_roll * sin_3 - cos_pitch * cos_3 * sin_roll),
										-cos_roll * sin_pitch, 	(cos_3 * sin_roll + cos_roll * cos_pitch * sin_3), 	(cos_roll * cos_pitch * cos_3 - sin_roll * sin_3);
				 				   					
				}
				break;
			}
			case(PointCloudTransformer::angleOrder::PRP):
			{
				if (rotation_frame == PointCloudTransformer::rotationFrame::WORLD)
				{
					//todo: can I represent a world RPY using body-fixed rotations?
					std::cout << "Warning: world-fixed transforms not yet supported\n";
				} else
				{
					rotation_matrix << 	(cos_pitch * cos_3 - cos_roll * sin_pitch * sin_3), 	sin_pitch * sin_roll, 	(cos_pitch * sin_3 + cos_roll * cos_3 * sin_pitch),
										sin_roll * sin_3, 										cos_roll, 				-cos_3 * sin_roll,
										(-cos_3 * sin_pitch - cos_pitch * cos_roll * sin_3), 	cos_pitch * sin_roll, 	(cos_pitch * cos_roll * cos_3 - sin_pitch * sin_3);
				 				   						
				}
				break;
			}
			case(PointCloudTransformer::angleOrder::PYP):
			{
				if (rotation_frame == PointCloudTransformer::rotationFrame::WORLD)
				{
					//todo: can I represent a world RPY using body-fixed rotations?
					std::cout << "Warning: world-fixed transforms not yet supported\n";
				} else
				{
					rotation_matrix << 	(cos_pitch * cos_yaw * cos_3 - sin_pitch * sin_3), 	-cos_pitch * sin_yaw, 	(cos_3 * sin_pitch + cos_pitch * cos_yaw * sin_3),
										cos_3 * sin_yaw, 									cos_yaw, 				sin_yaw * sin_3,
										(-cos_pitch * sin_3 - cos_yaw * cos_3 * sin_pitch), sin_pitch * sin_yaw, 	(cos_pitch * cos_3 - cos_yaw * sin_pitch * sin_3);
				 				   						
				}
				break;
			}
			case(PointCloudTransformer::angleOrder::YPY):
			{
				if (rotation_frame == PointCloudTransformer::rotationFrame::WORLD)
				{
					//todo: can I represent a world RPY using body-fixed rotations?
					std::cout << "Warning: world-fixed transforms not yet supported\n";
				} else
				{
					rotation_matrix << 	(cos_yaw * cos_pitch * cos_3 - sin_yaw * sin_3), 	(-cos_3 * sin_yaw - cos_yaw * cos_pitch * sin_3), 	cos_yaw * sin_pitch,
										(cos_yaw * sin_3 + cos_pitch * cos_3 * sin_yaw), 	(cos_yaw * cos_3 - cos_pitch * sin_yaw * sin_3), 	sin_yaw * sin_pitch,
										-cos_3 * sin_pitch, 								sin_pitch * sin_3, 									cos_pitch;
				 				   						
				}
				break;
			}
			case(PointCloudTransformer::angleOrder::YRY):
			{
				if (rotation_frame == PointCloudTransformer::rotationFrame::WORLD)
				{
					//todo: can I represent a world RPY using body-fixed rotations?
					std::cout << "Warning: world-fixed transforms not yet supported\n";
				} else
				{
					rotation_matrix << 	(cos_yaw * cos_3 - cos_roll * sin_yaw * sin_3), 	(-cos_yaw * sin_3 - cos_roll * cos_3 * sin_yaw), 	sin_yaw * sin_roll,
										(cos_3 * sin_yaw + cos_yaw * cos_roll * sin_3), 	(cos_yaw * cos_roll * cos_3 - sin_yaw * sin_3), 	-cos_yaw * sin_roll,
										sin_roll * sin_3, 									cos_3 * sin_roll, 									cos_roll;
				 				   						
				}
				break;
			}
			case(PointCloudTransformer::angleOrder::RYP):
			{
				if (rotation_frame == PointCloudTransformer::rotationFrame::WORLD)
				{
					//todo: can I represent a world RPY using body-fixed rotations?
					std::cout << "Warning: world-fixed transforms not yet supported\n";
				} else
				{
					rotation_matrix << 	cos_yaw * cos_pitch, 										-sin_yaw, 				cos_yaw * sin_pitch,
										(sin_roll * sin_pitch + cos_roll * cos_pitch * sin_yaw), 	cos_roll * cos_yaw, 	(cos_roll * sin_yaw * sin_pitch - cos_pitch * sin_roll),
										(cos_pitch * sin_roll * sin_yaw - cos_roll * sin_pitch), 	cos_yaw * sin_roll, 	(cos_roll * cos_pitch + sin_roll * sin_yaw * sin_pitch);			   						
				}
				break;
			}
			case(PointCloudTransformer::angleOrder::RPY):
			{
				if (rotation_frame == PointCloudTransformer::rotationFrame::WORLD)
				{
					//todo: can I represent a world RPY using body-fixed rotations?
					std::cout << "Warning: world-fixed transforms not yet supported\n";
				} else
				{
					//Body-fixed rotation requested!
					rotation_matrix << 	cos_pitch * cos_yaw, 									-cos_pitch * sin_yaw, 									sin_pitch,
		                   				(cos_roll * sin_yaw + cos_yaw * sin_roll * sin_pitch), 	(cos_roll * cos_yaw - sin_roll * sin_pitch * sin_yaw), 	-cos_pitch * sin_roll,
		                   				(sin_roll * sin_yaw - cos_roll * cos_yaw * sin_pitch),	(cos_yaw * sin_roll + cos_roll * sin_pitch * sin_yaw),	cos_roll * cos_pitch;
				}
				break;
			}
			case(PointCloudTransformer::angleOrder::PRY):
			{
				if (rotation_frame == PointCloudTransformer::rotationFrame::WORLD)
				{
					//todo: can I represent a world RPY using body-fixed rotations?
					std::cout << "Warning: world-fixed transforms not yet supported\n";
				} else
				{
					//Body-fixed rotation requested!
					rotation_matrix <<	(cos_pitch * cos_yaw + sin_pitch * sin_roll * sin_yaw), 	(cos_yaw * sin_pitch * sin_roll - cos_pitch * sin_yaw), 	cos_roll * sin_pitch,
										cos_roll * sin_yaw, 										cos_roll * cos_yaw, 											- sin_roll,
										(cos_pitch * sin_roll * sin_yaw - cos_yaw * sin_pitch), 	(cos_pitch * cos_yaw * sin_roll + sin_pitch * sin_yaw), 		cos_pitch * cos_roll;
		        }
				break;
			}
			case(PointCloudTransformer::angleOrder::PYR):
			{
				if (rotation_frame == PointCloudTransformer::rotationFrame::WORLD)
				{
					//todo: can I represent a world RPY using body-fixed rotations?
					std::cout << "Warning: world-fixed transforms not yet supported\n";
				} else
				{
					//Body-fixed rotation requested!
					rotation_matrix << 	cos_pitch * cos_yaw, 	(sin_pitch * sin_roll - cos_pitch * cos_roll * sin_yaw), 	(cos_roll * sin_pitch + cos_pitch * sin_yaw * sin_roll),
										sin_yaw, 				cos_yaw * cos_roll, 										-cos_yaw * sin_roll,
										-cos_yaw * sin_pitch, 	(cos_pitch * sin_roll + cos_roll * sin_pitch * sin_yaw), 	(cos_pitch * cos_roll - sin_pitch * sin_yaw * sin_roll);
				}
				break;
			}
			case(PointCloudTransformer::angleOrder::YPR):
			{
				if (rotation_frame == PointCloudTransformer::rotationFrame::WORLD)
				{
					//todo: can I represent a world RPY using body-fixed rotations?
					std::cout << "Warning: world-fixed transforms not yet supported\n";
				} else
				{
					//Body-fixed rotation requested!
					rotation_matrix << 	cos_yaw * cos_pitch,	(cos_yaw * sin_pitch * sin_roll - cos_roll * sin_yaw), 	(sin_yaw * sin_roll + cos_yaw * cos_roll * sin_pitch),
										cos_pitch * sin_yaw, 	(cos_yaw * cos_roll + sin_yaw * sin_pitch * sin_roll), 	(cos_roll * sin_yaw * sin_pitch - cos_yaw * sin_roll),
										-sin_pitch, 			cos_pitch * sin_roll, 									cos_pitch * cos_roll;
		        }
				break;
			}
			case(PointCloudTransformer::angleOrder::YRP):
			{
				if (rotation_frame == PointCloudTransformer::rotationFrame::WORLD)
				{
					//todo: can I represent a world RPY using body-fixed rotations?
					std::cout << "Warning: world-fixed transforms not yet supported\n";
				} else
				{
					//Body-fixed rotation requested!
					rotation_matrix <<	(cos_yaw * cos_pitch - sin_yaw * sin_roll * sin_pitch), 	-cos_roll * sin_yaw,	(cos_yaw * sin_pitch + cos_pitch * sin_yaw * sin_roll),
										(cos_pitch * sin_yaw + cos_yaw * sin_roll * sin_pitch), 	cos_yaw * cos_roll, 	(sin_yaw * sin_pitch - cos_yaw * cos_pitch * sin_roll),
										-cos_roll * sin_pitch, 										sin_roll, 				cos_roll * cos_pitch;
		        }
				break;
			}
			
			default:
			{
				//TODO: should not get here!
				std::cout << "Warning! invalid rotation order requested. Returning identity transform.\n";
			}
		}

		//Translation matrix:
		Eigen::Vector3f translation_matrix;
		translation_matrix << transform.x, transform.y, transform.z;

		if (translate_first)
		{
			translation_matrix = rotation_matrix * translation_matrix; //TODO: proof
		}

		transform_matrix << rotation_matrix.row(0), transform.x,
					 rotation_matrix.row(1), 		transform.y,
					 rotation_matrix.row(2), 		transform.z,
					 0, 0, 0, 				 		1;

	}

} //end namespace pointcloud_utils