#pragma once

#include "RobotData.h"
class DenavitHartenbergJoint
{
public:
	DenavitHartenbergMatrix Matrix; /**< @brief Gets or sets the current <see cref="DenavitHartenbergMatrix"/> associated with this joint. */
	DenavitHartenbergParameters Parameters; /** @brief Gets or sets the parameters for this joint. */

	/** @brief
	*   Updates the joint transformation matrix
	*   given a model transform matrix and reference position.
	*/
	DenavitHartenbergJoint() :
		Matrix(), Parameters(0, 0, 0, 0)
	{
	}
	~DenavitHartenbergJoint()
	{
	};
	/** @brief
	*   Initializes a new instance of the <see cref="DenavitHartenbergJoint"/> class.
	*
	* <param name="parameters">The <see cref="DenavitHartenbergParameters">
	* parameters </see> to be used to create the joint.</param>
	*/
	DenavitHartenbergJoint(const DenavitHartenbergParameters& parameters) :
		Matrix(), Parameters(parameters)
	{
	}
	/** @brief
	*   Initializes a new instance of the <see cref="DenavitHartenbergJoint"/> class.
	*
	* <param name="alpha">Angle in radians on the Z axis relatively to the last joint.</param>
	* <param name="a">Length or radius of the joint.</param>
	* <param name="theta">Angle in radians on the X axis relatively to the last joint.</param>
	* <param name="d">Offset along Z axis relatively to the last joint.</param>
	*/
	DenavitHartenbergJoint(double alpha, double a, double theta, double d) :
		Matrix(), Parameters(alpha, a, theta, d)
	{
	}

	dmatrix Compute(dmatrix transformMatrix)
	{
		// Calculate the new joint
		Matrix.Compute(Parameters);

		// Calculate the joint transformation matrix relative to the full model
		transformMatrix = transformMatrix * Matrix.Transform;

		return transformMatrix; // Return the new matrix, to pass along the chain
	}

};

