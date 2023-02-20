package frc.robot.Objects.Helpers;

class IKhelper {

	double theta1 = 0;
	double theta2 = 0;

	double inverseTheta1 = 0;
	double inverseTheta2 = 0;

	double offset1 = 0;
	double offset2 = 0;

	double length1 = 0;
	double length2 = 0;

	public IKhelper(double segOneThetaOffset, double segOneLength, double segTwoThetaOffset, double segTwoLength) {
		offset1 = segOneThetaOffset;
		offset2 = segTwoThetaOffset;

		length1 = segOneLength;
		length2 = segTwoLength;
	}

	public void setTarget(double inputX, double inputY) {

		double x = inputX;
		double y = inputY;

		double targetDistance = Math.sqrt(x * x + y * y);
		double maxLength = length1 + length2;
		double minLength = Math.abs(length1 - length2);

		if (targetDistance > maxLength) {
			x = (x / targetDistance) * maxLength;
			y = (y / targetDistance) * maxLength;
			theta1 = Math.atan2(x, y);
			theta2 = theta1;
			inverseTheta1 = theta1;
			inverseTheta2 = theta2;
			return;
		} else if (targetDistance < minLength) {
			x = (x / targetDistance) * minLength;
			y = (y / targetDistance) * minLength;
			theta1 =  Math.atan2(x, y);
			theta2 = theta1 + Math.PI;
			inverseTheta1 = theta1;
			inverseTheta2 = theta2;
			return;
		}

		double tempTheta1 =  (length1 * length1 + targetDistance * targetDistance - length2 * length2)
				/ (2 * length1 * targetDistance);
		double tempTheta2 =  (length1 * length1 + length2 * length2 - targetDistance * targetDistance)
				/ (2 * length1 * length2);

		theta1 = Math.acos(tempTheta1);
		theta1 += Math.atan2(x, y);
		theta2 = Math.acos(tempTheta2);
		theta2 += theta1 + Math.PI;

		double dir =  Math.atan2(x, y);

		inverseTheta1 = dir + -(theta1 - dir);
		inverseTheta2 = dir + -(theta2 - dir);

	}

	public double getTheta1() {
		return theta1 - offset1;
	}

	public double getTheta2() {
		return theta2 - offset2;
	}

	public double getInverseTheta1() {
		return inverseTheta1 - offset1;
	}

	public double getInverseTheta2() {
		return inverseTheta2 - offset2;
	}

}