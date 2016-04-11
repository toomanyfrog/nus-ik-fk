/* CS3242 3D Modeling and Animation
 * Programming Assignment II
 * School of Computing
 * National University of Singapore
 */
 
#include "BVHAnimator.h"

#include <ctime>

#include <iostream>
using namespace std;

// color used for rendering, in RGB
float color[3];

// convert a matrix to an array
static void mat4ToGLdouble16(GLdouble* m, glm::mat4 mat){
	// OpenGL matrix is column-major.
	for (uint i = 0; i < 4; i++)
		for (uint j = 0; j < 4; j++)
			m[i*4+j] = mat[i][j];
}

static glm::mat4 rigidToMat4(RigidTransform t) {
    glm::mat4 translation_mat = glm::translate(glm::mat4(1.0f), t.translation);
    glm::mat4 rotation_mat = glm::mat4_cast(t.quaternion);
    return translation_mat * rotation_mat;
}

static void rigidToGLdouble16(GLdouble *m, RigidTransform t) {
    mat4ToGLdouble16(m, rigidToMat4(t));
}

void renderSphere( float x, float y, float z, float size){		
	float radius = size;
	int numSlices = 32;
	int numStacks = 8; 
	static GLUquadricObj *quad_obj = gluNewQuadric();
	gluQuadricDrawStyle( quad_obj, GLU_FILL );
	gluQuadricNormals( quad_obj, GLU_SMOOTH );

	glPushMatrix();	
	glTranslated( x, y, z );

	glColor3f(color[0],color[1],color[2]);

	gluSphere(quad_obj,radius,numSlices,numStacks);	

	glPopMatrix();
}

void renderCyl( float x, float y, float z, float size){		
	float radius = size;
	int numSlices = 32;
	int numStacks = 8; 
	static GLUquadricObj *quad_obj = gluNewQuadric();
	gluQuadricDrawStyle( quad_obj, GLU_FILL );
	gluQuadricNormals( quad_obj, GLU_SMOOTH );

	glPushMatrix();	
	glTranslated( x, y, z );

	glColor3f(color[0],color[1],color[2]);

	gluCylinder(quad_obj,radius,radius,0.1, numSlices,numStacks);	

	glPopMatrix();
}


/**
 * Draw a bone from (x0,y0,z0) to (x1,y1,z1) as a cylinder of radius (boneSize)
 */
void renderBone( float x0, float y0, float z0, float x1, float y1, float z1, float boneSize ){
	GLdouble  dir_x = x1 - x0;
	GLdouble  dir_y = y1 - y0;
	GLdouble  dir_z = z1 - z0;
	GLdouble  bone_length = sqrt( dir_x*dir_x + dir_y*dir_y + dir_z*dir_z );

	static GLUquadricObj *  quad_obj = NULL;
	if ( quad_obj == NULL )
		quad_obj = gluNewQuadric();	
	gluQuadricDrawStyle( quad_obj, GLU_FILL );
	gluQuadricNormals( quad_obj, GLU_SMOOTH );

	glPushMatrix();

	glTranslated( x0, y0, z0 );

	double  length;
	length = sqrt( dir_x*dir_x + dir_y*dir_y + dir_z*dir_z );
	if ( length < 0.0001 ) { 
		dir_x = 0.0; dir_y = 0.0; dir_z = 1.0;  length = 1.0;
	}
	dir_x /= length;  dir_y /= length;  dir_z /= length;

	GLdouble  up_x, up_y, up_z;
	up_x = 0.0;
	up_y = 1.0;
	up_z = 0.0;

	double  side_x, side_y, side_z;
	side_x = up_y * dir_z - up_z * dir_y;
	side_y = up_z * dir_x - up_x * dir_z;
	side_z = up_x * dir_y - up_y * dir_x;

	length = sqrt( side_x*side_x + side_y*side_y + side_z*side_z );
	if ( length < 0.0001 ) {
		side_x = 1.0; side_y = 0.0; side_z = 0.0;  length = 1.0;
	}
	side_x /= length;  side_y /= length;  side_z /= length;

	up_x = dir_y * side_z - dir_z * side_y;
	up_y = dir_z * side_x - dir_x * side_z;
	up_z = dir_x * side_y - dir_y * side_x;

	GLdouble  m[16] = { side_x, side_y, side_z, 0.0,
	                    up_x,   up_y,   up_z,   0.0,
	                    dir_x,  dir_y,  dir_z,  0.0,
	                    0.0,    0.0,    0.0,    1.0 };
	glMultMatrixd( m );

	GLdouble radius= boneSize; 
	GLdouble slices = 8.0; 
	GLdouble stack = 3.0;  
	glColor3f(color[0],color[1],color[2]);
	gluCylinder( quad_obj, radius, radius, bone_length, slices, stack ); 

	glPopMatrix();
}

BVHAnimator::BVHAnimator(BVH* bvh)
{
	_bvh = bvh;
	setPointers();
}

void  BVHAnimator::renderFigure( int frame_no, float scale, int flag )
{	
	switch (flag){
	case 0:
		renderJointsGL( _bvh->getRootJoint(), _bvh->getMotionDataPtr(frame_no), scale);
	    break;
    case 1:
        renderJointsMatrix(frame_no, scale);
        break;
    case 2:
		renderJointsQuaternion(frame_no, scale);
		break;
	case 3:
		renderSkeleton( _bvh->getRootJoint(), _bvh->getMotionDataPtr(frame_no), scale );	
		break;
	case 4:	
		renderMannequin(frame_no,scale);
		break;
	default:
		break;
	}
}

void  BVHAnimator::renderSkeleton( const JOINT* joint, const float*data, float scale )
{	
	color[0] = 1.;
    color[1] = 0.;
    color[2] = 0.;

	float bonesize = 0.03;
	
    glPushMatrix();
	
    // translate
	if ( joint->parent == NULL )    // root
	{
		glTranslatef( data[0] * scale, data[1] * scale, data[2] * scale );
	}
	else
	{
		glTranslatef( joint->offset.x*scale, joint->offset.y*scale, joint->offset.z*scale );
	}

	// rotate
	for ( uint i=0; i<joint->channels.size(); i++ )
	{
		CHANNEL *channel = joint->channels[i];
		if ( channel->type == X_ROTATION )
			glRotatef( data[channel->index], 1.0f, 0.0f, 0.0f );
		else if ( channel->type == Y_ROTATION )
			glRotatef( data[channel->index], 0.0f, 1.0f, 0.0f );
		else if ( channel->type == Z_ROTATION )
			glRotatef( data[channel->index], 0.0f, 0.0f, 1.0f );
	}

	//end site? skip!
	if ( joint->children.size() == 0 && !joint->is_site)
	{
		renderBone(0.0f, 0.0f, 0.0f, joint->offset.x*scale, joint->offset.y*scale, joint->offset.z*scale,bonesize);
	}
	if ( joint->children.size() == 1 )
	{
		JOINT *  child = joint->children[ 0 ];
		renderBone(0.0f, 0.0f, 0.0f, child->offset.x*scale, child->offset.y*scale, child->offset.z*scale,bonesize);
	}
	if ( joint->children.size() > 1 )
	{
		float  center[ 3 ] = { 0.0f, 0.0f, 0.0f };
		for ( uint i=0; i<joint->children.size(); i++ )
		{
			JOINT *  child = joint->children[i];
			center[0] += child->offset.x;
			center[1] += child->offset.y;
			center[2] += child->offset.z;
		}
		center[0] /= joint->children.size() + 1;
		center[1] /= joint->children.size() + 1;
		center[2] /= joint->children.size() + 1;

		renderBone(	0.0f, 0.0f, 0.0f, center[0]*scale, center[1]*scale, center[2]*scale,bonesize);

		for ( uint i=0; i<joint->children.size(); i++ )
		{
			JOINT *  child = joint->children[i];
			renderBone(	center[0]*scale, center[1]*scale, center[2]*scale,
				child->offset.x*scale, child->offset.y*scale, child->offset.z*scale,bonesize);
		}
	}

	// recursively render all bones
	for ( uint i=0; i<joint->children.size(); i++ )
	{
		renderSkeleton( joint->children[ i ], data, scale );
	}
	glPopMatrix();
}

void  BVHAnimator::renderJointsGL( const JOINT* joint, const float*data, float scale )
{	
    // --------------------------------------
    // TODO: [Part 2a - Forward Kinematics]
    // --------------------------------------
    
	color[0] = 1.; color[1] = 0.; color[2] = 0.;

	glPushMatrix();

	// translate
	if ( joint->parent == NULL )    // root
	{
		glTranslatef( data[0] * scale, data[1] * scale, data[2] * scale );
	}
	else
	{
		glTranslatef( joint->offset.x*scale, joint->offset.y*scale, joint->offset.z*scale );
	}
	
	// rotate
	for ( uint i=0; i<joint->channels.size(); i++ )
	{
		CHANNEL *channel = joint->channels[i];
		if ( channel->type == X_ROTATION )
			glRotatef( data[channel->index], 1.0f, 0.0f, 0.0f );
		else if ( channel->type == Y_ROTATION )
			glRotatef( data[channel->index], 0.0f, 1.0f, 0.0f );
		else if ( channel->type == Z_ROTATION )
			glRotatef( data[channel->index], 0.0f, 0.0f, 1.0f );
	}


	// end site
	if ( joint->children.size() == 0 )
	{
		renderSphere(joint->offset.x*scale, joint->offset.y*scale, joint->offset.z*scale,0.07);
	}
	if ( joint->children.size() == 1 )
	{
		JOINT *  child = joint->children[ 0 ];
		renderSphere(child->offset.x*scale, child->offset.y*scale, child->offset.z*scale,0.07);
	}
	if ( joint->children.size() > 1 )
	{
		float  center[ 3 ] = { 0.0f, 0.0f, 0.0f };
		for ( uint i=0; i<joint->children.size(); i++ )
		{
			JOINT *  child = joint->children[i];
			center[0] += child->offset.x;
			center[1] += child->offset.y;
			center[2] += child->offset.z;
		}
		center[0] /= joint->children.size() + 1;
		center[1] /= joint->children.size() + 1;
		center[2] /= joint->children.size() + 1;

		renderSphere(center[0]*scale, center[1]*scale, center[2]*scale,0.07);

		for ( uint i=0; i<joint->children.size(); i++ )
		{
			JOINT *  child = joint->children[i];
			renderSphere(child->offset.x*scale, child->offset.y*scale, child->offset.z*scale,0.07);
		}
	}

	// recursively render all joints
	for ( uint i=0; i<joint->children.size(); i++ )
	{
		renderJointsGL( joint->children[i], data, scale );
	}
	glPopMatrix();
}


void  BVHAnimator::renderJointsMatrix( int frame, float scale )
{
	_bvh->matrixMoveTo(frame, scale);
	std::vector<JOINT*> jointList = _bvh->getJointList();	
	for(std::vector<JOINT*>::iterator it = jointList.begin(); it != jointList.end(); it++)
	{
		glPushMatrix();	
                
        GLdouble m[16];                  
		mat4ToGLdouble16(m, (*it)->matrix);
		glMultMatrixd(m);

		if ((*it)->is_site) {
			color[0] = 0.; color[1] = 1.; color[2] = 0.;
			renderSphere(0, 0, 0, 0.04);
		}
		else{
			if ((*it) == larm || (*it) == lforearm) {
				color[0] = 1.; color[1] = 0.5; color[2] = 0.;
			}
			else {
				color[0] = 0.; color[1] = 1.; color[2] = 0.;
			}
			renderSphere(0, 0, 0, 0.07);
		}

		glPopMatrix();
	}	
}

void  BVHAnimator::renderJointsQuaternion( int frame, float scale )
{
	_bvh->quaternionMoveTo(frame, scale);
	std::vector<JOINT*> jointList = _bvh->getJointList();	
	for(std::vector<JOINT*>::iterator it = jointList.begin(); it != jointList.end(); it++)
	{
		glPushMatrix();	

        // convert quaternion and translation into matrix for rendering        
        glm::mat4 mat = rigidToMat4((*it)->transform);
        GLdouble m[16];                  
		mat4ToGLdouble16(m, mat);
		glMultMatrixd(m);

		if ((*it)->is_site) {
			color[0] = 0.; color[1] = 0.; color[2] = 1.;
			renderSphere(0, 0, 0, 0.04);
		} else {
			color[0] = 0.; color[1] = 0.; color[2] = 1.;
			renderSphere(0, 0, 0, 0.07);
			//renderCyl(0, 0, 0, 0.07);
		}

		glPopMatrix();
	}	
}

void BVHAnimator::setPointers(){
	head = _bvh->getJoint(std::string(_BVH_HEAD_JOINT_));
	neck = _bvh->getJoint(std::string(_BVH_NECK_JOINT_));
	chest = _bvh->getJoint(std::string(_BVH_CHEST_JOINT_));
	spine = _bvh->getJoint(std::string(_BVH_SPINE_JOINT_));
	hip = _bvh->getJoint(std::string(_BVH_ROOT_JOINT_));   // root joint

	lshldr = _bvh->getJoint(std::string(_BVH_L_SHOULDER_JOINT_));
	larm = _bvh->getJoint(std::string(_BVH_L_ARM_JOINT_));
	lforearm = _bvh->getJoint(std::string(_BVH_L_FOREARM_JOINT_));
	lhand = _bvh->getJoint(std::string(_BVH_L_HAND_JOINT_));

	rshldr = _bvh->getJoint(std::string(_BVH_R_SHOULDER_JOINT_));
	rarm = _bvh->getJoint(std::string( _BVH_R_ARM_JOINT_));
	rforearm = _bvh->getJoint(std::string(_BVH_R_FOREARM_JOINT_));
	rhand = _bvh->getJoint(std::string(_BVH_R_HAND_JOINT_));

	lupleg = _bvh->getJoint(std::string(_BVH_L_THIGH_JOINT_));
	lleg = _bvh->getJoint(std::string(_BVH_L_SHIN_JOINT_));
	lfoot = _bvh->getJoint(std::string(_BVH_L_FOOT_JOINT_));
	ltoe = _bvh->getJoint(std::string(_BVH_L_TOE_JOINT_));

	rupleg = _bvh->getJoint(std::string(_BVH_R_THIGH_JOINT_));
	rleg = _bvh->getJoint(std::string(_BVH_R_SHIN_JOINT_));
	rfoot = _bvh->getJoint(std::string(_BVH_R_FOOT_JOINT_));
	rtoe = _bvh->getJoint(std::string(_BVH_R_TOE_JOINT_));
}

void getScaledOffset(OFFSET& c, OFFSET a, OFFSET b, float scale)
{
	c.x = (a.x-b.x)*scale;
	c.y = (a.y-b.y)*scale;
	c.z = (a.z-b.z)*scale;
}

void BVHAnimator::renderMannequin(int frame, float scale) {

    // --------------------------------------
    // TODO: [Part 2c - Forward Kinematics]
    // --------------------------------------
	// You can draw a couple of basic geometries to build the mannequin 
    // using the renderSphere() and renderBone() provided in BVHAnimator.cpp 
    // or GL functions like glutSolidCube(), etc.
    
    _bvh->quaternionMoveTo(frame, scale);
    //_bvh->matrixMoveTo(frame, scale);
    // NOTE: you can use matrix or quaternion to calculate the transformation
	std::vector<JOINT*> jointList = _bvh->getJointList();
	for (std::vector<JOINT*>::iterator it = jointList.begin(); it != jointList.end(); it++)
	{
		glPushMatrix();

		// convert quaternion and translation into matrix for rendering        
		glm::mat4 mat = rigidToMat4((*it)->transform);
		GLdouble m[16];
		mat4ToGLdouble16(m, mat);
		glMultMatrixd(m);
		if ((*it)->is_site) {
			color[0] = 0.; color[1] = 0.5; color[2] = 1.;
			renderSphere(0, 0, 0, 0.09); 
		}
		else {
			
			if ((*it) == head) { renderSphere(0, 0, 0, 0.15); }
			else if ((*it) == rhand || (*it) == lhand || (*it) == rforearm || (*it) == lforearm ) {
				color[0] = 0.; color[1] = 0.5; color[2] = 1.;
				if ((*it) == larm || (*it) == lforearm) {
					color[0] = 1.; color[1] = 0.5; color[2] = 0.;
				}
				renderSphere(0, 0, 0, 0.07);
			}
			else { 
				color[0] = 0.; color[1] = 0.5; color[2] = 1.;
				renderSphere(0, 0, 0, 0.09); 
			}
		}
		
		if ((*it)->children.size() == 0)
		{
			renderBone(0.0f, 0.0f, 0.0f, (*it)->offset.x*scale, (*it)->offset.y*scale, (*it)->offset.z*scale, 0.05);
		}
		else
		{
			JOINT *  child = (*it)->children[0];
			if ((*it) == chest || (*it) == spine) {
				renderBone(0.0f, 0.0f, 0.0f, child->offset.x*scale, child->offset.y*scale, child->offset.z*scale, 0.11);
			}
			else if ((*it) == lupleg || (*it) == rupleg || (*it) == lleg || (*it) == rleg)
			{
				renderBone(0.0f, 0.0f, 0.0f, child->offset.x*scale, child->offset.y*scale, child->offset.z*scale, 0.09);
			}
			else
			{
				renderBone(0.0f, 0.0f, 0.0f, child->offset.x*scale, child->offset.y*scale, child->offset.z*scale, 0.06);
			}
		}
		
		glPopMatrix();
	}

}

void BVHAnimator::solveLeftArm(int frame_no, float scale, float x, float y, float z)
{
    //_bvh->matrixMoveTo(frame_no, scale);      
    _bvh->quaternionMoveTo(frame_no, scale);
    // NOTE: you can use either matrix or quaternion to calculate the transformation


	float *LArx, *LAry, *LArz, *LFAry;

	float *mdata = _bvh->getMotionDataPtr(frame_no);
	// 3 channels - Xrotation, Yrotation, Zrotation
    // extract value address from motion data        
    CHANNEL *channel = larm->channels[0];
	LArx = &mdata[channel->index];

	channel = larm->channels[1];
	LAry = &mdata[channel->index];
	
	channel = larm->channels[2];
	LArz = &mdata[channel->index];
	
	channel = lforearm->channels[1];
	LFAry = &mdata[channel->index];

    cout << "Solving inverse kinematics..." << endl;
    clock_t start_time = clock();

    // -------------------------------------------------------
    // TODO: [Part 3] - Inverse Kinematics
    //
    // Put your code below
    // -------------------------------------------------------


	// REFERENCE: https://www.math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf

	float delta = 0.0001;					// for approximating the derivative
	int iterations = 20;					// number of times to iterate
	float damp_lambda = 0.009;				// damping constant chosen at random and seems to work
	float del_x = x - lhand->transform.translation.x;	// vector from hand to target
	float del_y = y - lhand->transform.translation.y;	
	float del_z = z - lhand->transform.translation.z;

	while ( iterations > 0 ) {

		_bvh->quaternionMoveTo(frame_no, scale);
		// Get current end effector position.
		glm::vec3 old_pos = lhand->transform.translation;

		*LArx = *LArx + delta;
		_bvh->quaternionMoveTo(frame_no, scale); 
		glm::vec3 new_pos = lhand->transform.translation;
		float dx_darx = (new_pos.x - old_pos.x) / delta;
		float dy_darx = (new_pos.y - old_pos.y) / delta;
		float dz_darx = (new_pos.z - old_pos.z) / delta;
		*LArx = *LArx - delta;

		*LAry = *LAry + delta;
		_bvh->quaternionMoveTo(frame_no, scale);
		new_pos = lhand->transform.translation;
		float dx_dary = (new_pos.x - old_pos.x) / delta;
		float dy_dary = (new_pos.y - old_pos.y) / delta;
		float dz_dary = (new_pos.z - old_pos.z) / delta;
		*LAry = *LAry - delta;

		*LArz = *LArz + delta;
		_bvh->quaternionMoveTo(frame_no, scale);
		new_pos = lhand->transform.translation;
		float dx_darz = (new_pos.x - old_pos.x) / delta;
		float dy_darz = (new_pos.y - old_pos.y) / delta;
		float dz_darz = (new_pos.z - old_pos.z) / delta;
		*LArz = *LArz - delta;

		*LFAry = *LFAry + delta;
		_bvh->quaternionMoveTo(frame_no, scale);
		new_pos = lhand->transform.translation;
		float dx_dfary = (new_pos.x - old_pos.x) / delta;
		float dy_dfary = (new_pos.y - old_pos.y) / delta;
		float dz_dfary = (new_pos.z - old_pos.z) / delta;
		*LFAry = *LFAry - delta;

		glm::mat3x4 jacobian = glm::mat3x4(dx_darx, dx_dary, dx_darz, dx_dfary,
			dy_darx, dy_dary, dy_darz, dy_dfary,
			dz_darx, dz_dary, dz_darz, dz_dfary);
		glm::mat4x3 transpose = glm::transpose(jacobian);
		glm::vec3 de = glm::vec3(del_x, del_y, del_z);

		// Damped least squares method
		glm::mat3x3 jjt = transpose * jacobian;
		glm::mat3x3 inversed = jjt + powf(damp_lambda, 2.) * glm::mat3x3();
		inversed = glm::inverse(inversed);
		glm::mat4x3 dls = inversed * transpose;

		glm::vec4 d_theta = de * dls;
		
		*LArx = *LArx + d_theta[0];
		*LAry = *LAry + d_theta[1];
		*LArz = *LArz + d_theta[2];
		*LFAry = *LFAry + d_theta[3];

		_bvh->quaternionMoveTo(frame_no, scale);

		del_x = x - lhand->transform.translation.x;	// vector from hand to target
		del_y = y - lhand->transform.translation.y;
		del_z = z - lhand->transform.translation.z;

		iterations--;

	}
	
	// ----------------------------------------------------------
    // Do not touch
    // ----------------------------------------------------------
    clock_t end_time = clock();
    float elapsed = (end_time - start_time) / (float)CLOCKS_PER_SEC;
    cout << "Solving done in " << elapsed * 1000 << " ms." << endl;
}