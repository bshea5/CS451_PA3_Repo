//------------------------------------------------------------------------------
//  Copyright 2007-2014 by Jyh-Ming Lien and George Mason University
//  See the file "LICENSE" for more information
//------------------------------------------------------------------------------


#pragma once

#include "objReader.h"
#include "model.h"
#include "Library.hpp" //motion capture library
#include "pose_utils.hpp"
#include <list>
#include <float.h>
using namespace std;

//-----------------------------------------------------------------------------
// INPUTS
list<string> input_filenames;
string mocap_dir; //motion capture dir

//-----------------------------------------------------------------------------
// Intermediate data
list<model> models; //NOTE: only the first model of the list is used in this code
float R=0;          //radius
Point3d COM;        //center of mass

int currentMotion=0; //current motion dataset, used to animate skeleton
int currentFrame=0;  //current frame in the dataset, used to animate skeleton

//-----------------------------------------------------------------------------

/////------ PA3 Variables START-----

bool BindToSkin = false;       // initially the skeleton is not binded to the skin (mesh)

Character::Pose BindingPose;   // at this pose that the skeleton binds to the skin

vector< vector<double> > SkinningWeights; //weights for each vertex and each bone
                                          //there are N vector<double>, where N is the number of vertices in the mesh
                                          //there are K double in vector<double>, where K is the number of bones in skeleton 

vector< vector<Point3d> > BoneSpaceCoordinates; //local coordinates for each vertex in bone subspace
                                                //there are N vector<Point3d>, where N is the number of vertices in the mesh
                                                //there are K double in vector<Point3d>, where K is the number of bones in skeleton 

////------ PA3 Variables END-----

//-----------------------------------------------------------------------------

/////------ PA3 TODOs START-----

//TODO: implement this function to setup the binding pose.
//      See details below
void setupBindingPose();

//TODO: implement this function to bind the skeleton to the skin.
//      See details below
void bind2skin();

//TODO: skeleton-subspace deformation. perform SSD 
void SSD();


/////------ PA3 TODOs END-----

//-----------------------------------------------------------------------------
bool readfromfile();
void computeCOM_R();

//-----------------------------------------------------------------------------
bool parseArg(int argc, char ** argv)
{
    for(int i=1;i<argc;i++){
        if(argv[i][0]=='-')
        {
			if (string(argv[i]) == "-mocap")      mocap_dir = argv[++i];
			else
				return false; //unknown
        }
        else{
            input_filenames.push_back(argv[i]);
        }
    }

    return true;
}

void printUsage(char * name)
{
    //int offset=20;
    cerr<<"Usage: "<<name<<" [options] -mocap dir *.obj \n"
        <<"options:\n\n";
    cerr<<"\n-- Report bugs to: Jyh-Ming Lien jmlien@gmu.edu"<<endl;
}

//-----------------------------------------------------------------------------

bool readfromfiles()
{
	if (input_filenames.empty())
	{
		cerr << "! Error: No input model" << endl;
		return false;
	}

	if (mocap_dir.empty())
	{
		cerr << "! Error: No input motion capture data" << endl;
		return false;
	}

	//read obj model
    long vsize=0;
    long fsize=0;

    uint id=0;
    for(list<string>::iterator i=input_filenames.begin();i!=input_filenames.end();i++,id++){
        cout<<"- ["<<id<<"/"<<input_filenames.size()<<"] Start reading "<<*i<<endl;
        model m;
        if(!m.build(*i)) continue;
        cout<<"- ["<<id<<"/"<<input_filenames.size()<<"] Done reading "<<m.v_size<<" vertices and "<<m.t_size<<" facets"<<endl;
        vsize+=m.v_size;
        fsize+=m.t_size;
        models.push_back(m);
    }
    cout<<"- Total: "<<vsize<<" vertices, "<<fsize<<" triangles, and "<<input_filenames.size()<<" models"<<endl;
    computeCOM_R();
	
	//read mocap skeleton and animation
	Library::init(mocap_dir);

	//setup binding pose
	setupBindingPose();

    return true;
}

void computeCOM_R()
{
    //compute a bbox
    double box[6]={FLT_MAX,-FLT_MAX,FLT_MAX,-FLT_MAX,FLT_MAX,-FLT_MAX};
    //-------------------------------------------------------------------------
    for(list<model>::iterator i=models.begin();i!=models.end();i++){
        for(unsigned int j=0;j<i->v_size;j++){
            Point3d& p=i->vertices[j].p;
            if(p[0]<box[0]) box[0]=p[0];
            if(p[0]>box[1]) box[1]=p[0];
            if(p[1]<box[2]) box[2]=p[1];
            if(p[1]>box[3]) box[3]=p[1];
            if(p[2]<box[4]) box[4]=p[2];
            if(p[2]>box[5]) box[5]=p[2];
        }//j
    }//i

    //-------------------------------------------------------------------------
    // compute center of mass and R...
    COM.set( (box[1]+box[0])/2,(box[3]+box[2])/2,(box[5]+box[4])/2);

    //-------------------------------------------------------------------------
	R=0;
    for(list<model>::iterator i=models.begin();i!=models.end();i++){
        for(unsigned int j=0;j<i->v_size;j++){
            Point3d& p=i->vertices[j].p;
            float d=(float)(p-COM).normsqr();
            if(d>R) R=d;
        }//j
    }//i

    R=sqrt(R);
}

void setupBindingPose()
{
	//set binding pose to zero pose
	Library::Motion const &mo = Library::motion(0);
	mo.get_pose(0, BindingPose);

	BindingPose.root_position = Vector3d(0, 0, 0);
	BindingPose.root_orientation = Quaternion();
	for (int k = 0; k < BindingPose.bone_orientations.size(); k++)
	{
		BindingPose.bone_orientations[k] = Quaternion();
	}

	//
	// DONE: Determine the Binding Pose, you can do it manually 
	// EXTRA: Do a GUI to move bones
	//
	//NOTES--------------------
	//0-4 his left leg
	//5-7 body
	//8-14 his left arm + hand
	//15 - 17 neck + head
	//18 - 24 right arm + hand
	//25-29 his right leg
	//-------------------------
	BindingPose.root_position[1] -= 0.5; //reposition root by some y direction
	//arms
	BindingPose.bone_orientations[8] = Quaternion::get(0.10, Vector3d(0, 1, 0));
	BindingPose.bone_orientations[9] = Quaternion::get(-0.10, Vector3d(0, 1, 0));
	BindingPose.bone_orientations[10] = Quaternion::get(-0.70, Vector3d(0, 1, 0));
	BindingPose.bone_orientations[18] = Quaternion::get(-0.15, Vector3d(0, 1, 0));
	BindingPose.bone_orientations[19] = Quaternion::get(0.10, Vector3d(0, 1, 0));
	BindingPose.bone_orientations[20] = Quaternion::get(0.70, Vector3d(0, 1, 0));
	//legs
	BindingPose.bone_orientations[0] = Quaternion::get(0.30, Vector3d(0, 0, 1));
	BindingPose.bone_orientations[1] =  Quaternion::get(-0.60, Vector3d(0, 0, 1));
	BindingPose.bone_orientations[25] = Quaternion::get(-0.30, Vector3d(0, 0, 1));
	BindingPose.bone_orientations[26] =  Quaternion::get(0.60, Vector3d(0, 0, 1));
}

//
// This function will be called when "B" (captial b) is pressed
//
void bind2skin()
{
	if (BindToSkin) return; //already binded
	
	using namespace Character;
	//work on the first model only
	model& model = models.front();
	
	//
	// TODO: compute SkinningWeights using BindingPose
	//
	//       Read amc_library/Lirary.hpp, amc_library/Skeleton.hpp
	//       to find out how to get all the bones
	//
	SkinningWeights.resize(model.v_size);
	
	//get information on bones
	WorldBones *wb = new WorldBones();
	get_world_bones(BindingPose, *wb);

	Library::Skeleton const *skeleton = BindingPose.skeleton;
	for (int i = 0; i < model.v_size; i++)
	{
		SkinningWeights[i].resize(skeleton->bones.size());
		vertex& v = model.vertices[i];
		float dist1 = NULL;
		float dist2 = NULL;
		float dist3 = NULL;
		int b1, b2, b3; //indices to the closed bones to v
		b1 = 0;
		b2 = 0;
		b3 = 0;
		for (int j = 0; j < skeleton->bones.size(); j++)
		{
			Vector3d base = wb->bases[j];
			//default all bones to zero for weight
			SkinningWeights[i][j] = 0;
			float dx, dy, dz, dist;
			dx = pow((v.p[0] - base[0]), 2);
			dy = pow((v.p[1] - base[1]), 2);
			dz = pow((v.p[2] - base[2]), 2);
			dist = sqrt(dx + dy + dz);
			//check for closest bones to model vertex
			if (dist3 == NULL || dist < dist3)
			{
				if (dist2 == NULL || dist < dist2)
				{
					if (dist1 == NULL || dist < dist1)
					{
						dist3 = dist2;
						dist2 = dist1;
						dist1 = dist;
						b1 = j; //save bone index for later
						//std::cout << "set b1: " << b1 << std::endl;
					}
					else
					{
						dist3 = dist2;
						dist2 = dist;
						b2 = j;
						//std::cout << "set b2: " << b2 << std::endl;
					}
				}
				else
				{
					dist3 = dist;
					b3 = j;
					//std::cout << "set b3: " << b3 << std::endl;
				}
			}

		}// end for each bone
		//assign weights for bones closest to v, all other bone weights are zero
		 std::cout << "closest bones to v-" << i << "are bones: " 
		 	<< b1 << ", " << b2 << ", " << b3 << "." << std::endl;


	}//	end for each vertex
	
	//
	// TODO: compute BoneSpaceCoordinates using BindingPose
	//       
	//       determine the cooridnates of each model vertex with respect to each bone
	//       in binding pose

	//
	BindToSkin = true;
}

//TODO: skeleton-subspace deformation. perform SSD 
void SSD()
{
	//
	//work on the first model only
	//
	model& model = models.front();

	//
	// recompute the position of each vertex in model
	// using BoneSpaceCoordinates and SkinningWeights
	//
}



//-----------------------------------------------------------------------------
//
//
//
//  Open GL stuff below
//
//
//-----------------------------------------------------------------------------

#include <draw.h>



