#pragma once

//transforming incapsule
class Transform{
public:
	float pos_x, pos_y, pos_z;
	float rot_x, rot_y, rot_z;
	float sca_x, sca_y, sca_z;
	float rotate_speed;
	float move_speed;

	Transform(float move_speed_v, float rot_speed_v):pos_x(0.0), pos_y(0.0),pos_z(0.0), rot_x(0.0), rot_y(0.0), rot_z(0.0), sca_x(0.2), sca_y(0.2), sca_z(0.2), rotate_speed(rot_speed_v), move_speed(move_speed_v){}

	void ProcessMove(float dir_x, float dir_y, float dir_z, float deltaTime) {
		pos_x += dir_x * move_speed * deltaTime;
		pos_y += dir_y * move_speed * deltaTime;
		pos_z += dir_z * move_speed * deltaTime;
	}

	void ProcessRotate(float xoffset, float yoffset)
	{
		xoffset *= rotate_speed;
		yoffset *= rotate_speed;
		rot_x += xoffset;
		rot_y += yoffset;
	}
};