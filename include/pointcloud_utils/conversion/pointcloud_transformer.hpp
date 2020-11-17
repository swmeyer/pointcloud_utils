/** Author: Stephanie Meyer swmeyer16@gmail.com 12 Nov 2020
 * Brief: Transformer class for point clouds - can handle many different rotation schema
 * File: pointcloud_transformer.hpp
 */

#ifndef POINTCLOUD_TRANSFORMER_HPP
#define POINTCLOUD_TRANSFORMER_HPP

// -------------------------------
#include "pointcloud_utils/pointcloud_utils.hpp"
#include <Eigen/Dense>
// -------------------------------

namespace pointcloud_utils
{

	class PointCloudTransformer
	{
		public:
			PointCloudTransformer();
			~PointCloudTransformer();

			enum angleOrder
			{
				RYR,
				RPR,
				PRP,
				PYP,
				YPY,
				YRY,
				RYP,
				RPY,
				PRY,
				PYR,
				YPR,
				YRP,
				INVALID
			};

			enum rotationFrame
			{
				WORLD,
				BODY
			};

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
			void transformCloud
			(
				std::vector<pointstruct>& cloud, 
				const Transform& transform, 
				const angleOrder& angle_order,
				const rotationFrame& rotation_frame, 
				const bool& fwd_transform,
				const bool& translate_first,
				const bool& publish_transform = false
			);

			/**
			 * @function 	getAngleOrderType
			 * @brief 		given a string of 3 letters representing the angle order,
			 *				determine which enumerated angle order is being requested
		 	 * @param 		str - the string containing the angle order request
		 	 * @return 		angleOrder - the enumerated value of the given angle order, 
		 	 *				or -1 if request was invalid
		 	 */
			angleOrder getAngleOrderType(const std::string& str);

			/**
			 * @function 	generateTransformMatrix
			 * @brief 		given the transform values and an angle order for the rotations to be applied in (rotation frame either local or world-fixed),
			 * 			    generate a 4x4 affine transform matrix that includes first the rotation by the angles about body-fixed axes and then the translation in
			 * 				the rotated frame
			 * @param 		transformMatrix - place to store the generated transform matrix
			 * @param 		transform - holder of the rotation and translation values to use
			 * @param 		angle_order - the order the given rotations should be applied in
			 * @param 		rotation_frame - the frame the user intended the angles to be applied in
			 * @param 		translate_first - if true, apply the translation in the original frame, otherwise, apply after rotation
	 		 * @return 		void
			 */
			void generateTransformMatrix
			(
				Eigen::MatrixXf& transformMatrix, 
				const Transform& transform, 
				const angleOrder& angle_order,
				const rotationFrame& rotation_frame,
				const bool& translate_first
			);
		
		private:

			

	}; //end class PointCloudTransformer
	

} //end namespace pointcloud_utils

#endif //end ifndef POINTCLOUD_TRANSFORMER_HPP