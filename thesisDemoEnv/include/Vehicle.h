#pragma once

#include <cmath>
#include <float.h>
#include "Clock.h"
#include "DetectedObject.h"
#include "Line.h"

class Vehicle
{
private:
	float _x,_y,_theta;
	float _halfWidth,_leftWheelConstant,_rightWheelConstant, _wheelRadius, _wheelBase;
	float _field_of_view, _max_range;
	double deltaTime;
	float LTL, LTR; // last tick left, last tick right 

public:
	Vehicle(float halfWidth = 0.0f, float leftWheelConstant = 0.0f, float rightWheelConstant = 0.0f, float field_of_view = 0.0f, float max_range = FLT_MAX,
				float wheelRadius = 0.0f, float wheelBase = 0.0f){
		_halfWidth = halfWidth;
		_leftWheelConstant = leftWheelConstant;
		_rightWheelConstant = rightWheelConstant;
		_wheelBase = wheelBase;
		_wheelRadius = wheelRadius;
		_x = FLT_MAX;
		_y = FLT_MAX;
		_theta = 0.0f;
		deltaTime = 0.005;
		// deltaTime = 0.02;
		LTL = 0;
		LTR = 0;
		_max_range = max_range;
		_field_of_view = field_of_view;
	}
	inline void SetLocation(float x, float y, float theta){
		_x = x;
		_y = y;
		_theta = theta;
	}
	inline void GetLocation (float& x, float& y, float& theta){
		x = _x;
		y = _y;
		theta = _theta;
	}
	inline void UpdateLocation(int leftWheelTick, int rightWheelTick){
		deltaTime = ElapsedTime();
		float vL = ((leftWheelTick - LTL)/deltaTime)*M_PI/180.0f;
		float vR = ((rightWheelTick - LTR)/deltaTime)*M_PI/180.0f;

		float Vl = vL * _wheelRadius; // converting from rad/s to cm/s
		float Vr = vR * _wheelRadius;

		float V = (Vl+Vr)/2.0f; // getting vehicle velocity
		float W = (Vr-Vl)/_wheelBase; // angular velocity of vehicle

		float thetaRadians = _theta*M_PI/180;

		// differential approximation
		float k00 = V*cos(thetaRadians);
		float k01 = V*sin(thetaRadians);
		float k02 = W;

		float k10 = V*cos(thetaRadians+deltaTime/2.0f*k02);
		float k11 = V*sin(thetaRadians+deltaTime/2.0f*k02);
		float k12 = W;

		float k20 = V*cos(thetaRadians+deltaTime/2.0f*k12);
		float k21 = V*sin(thetaRadians+deltaTime/2.0f*k12);
		float k22 = W;

		float k30 = V*cos(thetaRadians+deltaTime/2.0f*k22);
		float k31 = V*sin(thetaRadians+deltaTime/2.0f*k22);
		float k32 = W;

		// approximation results
		_x = _x + deltaTime/6.0f*(k00+2*(k10+k20)+k30);
		_y = _y + deltaTime/6.0f*(k01+2*(k11+k21)+k31);
		_theta = (thetaRadians + deltaTime/6.0f*(k02+2*(k12+k22)+k32))*(180.0/M_PI);

		printf("x=%f | y=%f | theta=%f | Dt=%f\n",_x, _y, _theta,deltaTime);
		ResetClock();
	}

	inline void Transform(float localX, float localY, float& globalX, float& globalY){

		float c = cos(_theta * M_PI / 180.0f);
		float s = sin(_theta * M_PI / 180.0f);
		globalX = c * localX - s * localY;
		globalY = s * localX + c * localY;
		globalX += _x;
		globalY += _y;
	}

	inline void TransformInverse(float globalX, float globalY, float& localX, float& localY){

		float c = cos(_theta * M_PI / 180.0f);
		float s = sin(_theta * M_PI / 180.0f);
		localX -= _x;
		localY -= _y;
		localX = c * globalX + s * globalY;
		localY = -s * globalX + c * globalY;
	}


	inline bool Detect(std::vector<DetectedObject>& objects, float& detectX, float& detectY, float& detectSqDist, float prevMinSqDist = 10000000)
	{

		float thetaRadians = _theta * M_PI / 180.0f;
		float thetaFovRadians1 = (_theta+_field_of_view) * M_PI / 180.0f;
		float thetaFovRadians2 = (_theta-_field_of_view) * M_PI / 180.0f;
		float minSqDist = prevMinSqDist;		// keep track of closest
		float minDetectX;
		float minDetectY;
		bool detected = false;

		printf("Running Detect Objs....%f\n", minSqDist);
		printf("Objects %i\n", objects.size());

		// -------- Obstacle Detection ----------- //
		for(int i = 0; i < objects.size(); i++)
		{
			// Compute distance to object
			float obj_x = objects[i].GetX();
			float obj_y = objects[i].GetY();
			float sqDist = ((obj_x - _x) * (obj_x - _x) + (obj_y - _y) * (obj_y - _y));

			if( (sqDist < _max_range * _max_range) && sqDist < minSqDist)
			{
				// Compute projection toward object
				float c = cos(thetaRadians);
				float s = sin(thetaRadians);
				float a = (obj_x - _x) / sqrtf(sqDist);
				float b = (obj_y - _y) / sqrtf(sqDist);
				float dot = c*a + s*b;

				//printf("Dot %f\n", dot);

				if(dot > cos(_field_of_view * M_PI / 180.0f))	// within cos(angle/2)
				{
					detected = true;
					minDetectX = objects[i].GetX();
					minDetectY = objects[i].GetY();	
					minSqDist = sqDist;	// update minimum
					printf("DETECTED OBJECT!!!\n");
					//printf("DETECTED OBJECT - X: %f| Y: %f| ID:%i\n", obj_x,obj_y, rover_detection1.GetObjectID());
				}
			}
		}


		// results ---------
		detectSqDist = minSqDist;
		if(detected)
		{			
			detectX = minDetectX;
			detectY = minDetectY;
		}

		return detected;
	}


	inline bool Detect(std::vector<Line>& lines, float& detectX, float& detectY, float& detectSqDist, float prevMinSqDist = 10000000)
	{

		float thetaRadians = _theta * M_PI / 180.0f;
		float thetaFovRadians1 = (_theta+_field_of_view) * M_PI / 180.0f;
		float thetaFovRadians2 = (_theta-_field_of_view) * M_PI / 180.0f;
		float minSqDist = prevMinSqDist;		// keep track of closest
		float minDetectX;
		float minDetectY;
		bool detected = false;

		printf("Running Detect Lines....%f\n", minSqDist);
		printf("Lines %i\n", lines.size());


		// -------- Line (Boundary) and Corner Detection ----------- //	
		bool hit1;
		bool hit2;
		float bx1,by1;
		float bx2,by2;

		// -------- Line (Boundary) Detection ----------- //	
		for(int i = 0; i < lines.size(); i++)
		{		
			float lx, ly, ldot;

			// ---- First Raycast (Left hand Field of View)
			bool hit = lines[i].Intersect(thetaFovRadians1, _x, _y, lx, ly);		
			if(hit)
			{
				float sqDist = ((lx-_x)*(lx-_x))+((ly-_y)*(ly-_y));

				//printf("Line HIT!!! %i, %f|%f  %f|%f\n", i, sqrtf(sqDist), _max_range, lx, ly);

				if( (sqDist < _max_range * _max_range) && sqDist < minSqDist)
				{
					// Compute projection toward detected point
					float c = cos(thetaRadians);
					float s = sin(thetaRadians);
					float a = (lx - _x) / sqrtf(sqDist);
					float b = (ly - _y) / sqrtf(sqDist);
					float dot = c*a + s*b;

					//printf("Dot %f\n", dot);

					//if(dot > cos(_field_of_view * M_PI / 180.0f))	// within cos(angle/2)
					if(dot > 0.0f)
					{
						// Track which hits are good
						hit1 = true;
						bx1 = lx;
						by1 = ly;

						detected = true;
						minDetectX = lx;
						minDetectY = ly;
						minSqDist = sqDist;	// update minimum		
						printf("DETECTED LINE!!!\n");			
						//printf("DETECTED LINE - X: %f| Y: %f| ID:%i\n", lx, ly, rover_detection1.GetObjectID());
					}	
				}
			}


			// ---- Second Raycast (Right hand Field of View)
			hit = lines[i].Intersect(thetaFovRadians2, _x, _y, lx, ly);
			if(hit)
			{
				float sqDist = ((lx-_x)*(lx-_x))+((ly-_y)*(ly-_y));

				//printf("Line HIT!!! %i, %f|%f  %f|%f\t", i, sqrtf(sqDist), max_range, lx, ly);

				if( (sqDist < _max_range * _max_range) && sqDist < minSqDist)
				{
					// Compute projection toward detected point
					float c = cos(thetaRadians);
					float s = sin(thetaRadians);
					float a = (lx - _x) / sqrtf(sqDist);
					float b = (ly - _y) / sqrtf(sqDist);
					float dot = c*a + s*b;

					//printf("Dot %f\n", dot);

					//if(dot > cos(_field_of_view * M_PI / 180.0f))	// within cos(angle/2)
					if(dot > 0.0f)	
					{
						// Track which hits are good
						hit2 = true;
						bx2 = lx;
						by2 = ly;

						detected = true;
						minDetectX = lx;
						minDetectY = ly;
						minSqDist = sqDist;	// update minimum
						//printf("DETECTED LINE - X: %f| Y: %f| ID:%i\n", lx, ly, rover_detection1.GetObjectID());
					}	
				}
			}
		}


		// ----- Corner detection (both rays hit) ------- //
		if(hit1 && hit2)
		{
			float bx = fabs(bx1) > fabs(bx2) ? bx1 : bx2;
			float by = fabs(by1) > fabs(by2) ? by1 : by2;
			float sqDist = ((bx-_x)*(bx-_x))+((by-_y)*(by-_y));

			minSqDist = sqDist;
			minDetectX = bx;
			minDetectY = by;	
		}


		// results ---------
		detectSqDist = minSqDist;
		if(detected)
		{			
			detectX = minDetectX;
			detectY = minDetectY;
		}

		return detected;
	}



	inline bool Detect(Vehicle& vehicle, float& detectX, float& detectY, float& detectSqDist, float prevMinSqDist = 10000000)
	{

		float thetaRadians = _theta * M_PI / 180.0f;
		float thetaFovRadians1 = (_theta+_field_of_view) * M_PI / 180.0f;
		float thetaFovRadians2 = (_theta-_field_of_view) * M_PI / 180.0f;
		float minSqDist = prevMinSqDist;		// keep track of closest
		float minDetectX;
		float minDetectY;
		bool detected = false;


		// -------- Detect other vehicle ----------- //
		// Compute distance to vehicle
		float vehicle_x, vehicle_y, vehicle_theta;
		vehicle.GetLocation(vehicle_x,vehicle_y,vehicle_theta);
		float sqDist = ((vehicle_x - _x) * (vehicle_x - _x) + (vehicle_y - _y) * (vehicle_y - _y));

		if( (sqDist < _max_range * _max_range) && sqDist < minSqDist)
		{
			// Compute projection toward object
			float c = cos(thetaRadians);
			float s = sin(thetaRadians);
			float a = (vehicle_x - _x) / sqrtf(sqDist);
			float b = (vehicle_y - _y) / sqrtf(sqDist);
			float dot = c*a + s*b;

			//printf("Dot %f\n", dot);

			if(dot > cos(_field_of_view * M_PI / 180.0f))	// within cos(angle/2)
			{
				detected = true;
				minDetectX = vehicle_x;
				minDetectY = vehicle_y;
				minSqDist = sqDist;	// update minimum
			}
		}

		// results ---------
		detectSqDist = minSqDist;
		if(detected)
		{			
			detectX = minDetectX;
			detectY = minDetectY;
		}

		return detected;
	}
};