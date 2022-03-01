// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Nic Olsen, Ruochun Zhang
// =============================================================================
// Chrono::Gpu demo using SMC method. A body whose geometry is described by an
// OBJ file is time-integrated in Chrono and interacts with a granular wave tank
// in Chrono::Gpu via the co-simulation framework. The entire simulation consists
// of 2 runs: the settling phase (which outputs a checkpoint file), and a restarted
// phase (which load the checkpoint file and then drop the ball, literally).
// =============================================================================

#include <iostream>
#include <vector>
#include <string>

#include "chrono/core/ChGlobal.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChForce.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/assets/ChCylinderShape.h"

#include "chrono_gpu/ChGpuData.h"
#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"
#include "chrono_gpu/utils/ChGpuVisualization.h"

#include "chrono_thirdparty/filesystem/path.h"
#include <iostream>
#include <vector>
#include <string>

#include "chrono/core/ChGlobal.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChForce.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/utils/ChUtilsSamplers.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/assets/ChCylinderShape.h"

#include "chrono_gpu/ChGpuData.h"
#include "chrono_gpu/physics/ChSystemGpu.h"
#include "chrono_gpu/utils/ChGpuJsonParser.h"
#include "chrono_gpu/utils/ChGpuVisualization.h"

#include "chrono_thirdparty/filesystem/path.h"

// me
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChCoordsys.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/motion_functions/ChFunction_Sine.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorLinearForce.h"
#include "chrono/physics/ChLinkMotorLinearDriveline.h"
#include "chrono/physics/ChLinkMotor.h"
#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChAsset.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono_irrlicht/ChApiIrr.h"
#include "chrono_irrlicht/ChIrrAppInterface.h"
#include "chrono_irrlicht/ChIrrAssetConverter.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"

//me
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
using namespace chrono;
using namespace chrono::gpu;

// Output frequency
float out_fps = 10;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = false;
float render_fps = 2000;





// read number of shperesfrom checkpointfile
//inter voulme of meshes
double VM = 100;
//detremin the Z from paraview
double ZB = 180;
double VoidRatio(int number, double Zbox, ChGpuSimulationParameters& params) {
     float Psphere_radius = params.sphere_radius;
    float vol_sphere = (number * (4 *3.14 * Psphere_radius * Psphere_radius * Psphere_radius))/3;
    float vol_total = (Zbox * params.box_X * params.box_Y);
    float vol_void = vol_total - vol_sphere;
    double voidRatio = vol_void / vol_sphere;
    float RDensity = (0.91 - voidRatio) / (0.91 - 0.35);
    std::cout << "The void Ratio Is: " << voidRatio << std::endl;
    std::cout << "The Relative Density Is: " << RDensity << std::endl;
    return voidRatio;



}

void cube(ChSystemGpuMesh& gpu_sys, ChGpuSimulationParameters& params) {

gpu_sys.AddMesh(GetChronoDataFile("models/cube.obj"), ChVector<float>(0), ChMatrix33<float>(150),
    500);
//gpu_sys.AddMesh(GetChronoDataFile("models/sphere.obj"), ChVector<float>(0), ChMatrix33<float>(sphere_radius),
//    sphere_mass);


// One more thing: we need to manually enable mesh in this run, because we disabled it in the settling phase,
// let's overload that option.
gpu_sys.EnableMeshCollision(true);

gpu_sys.Initialize();
std::cout << gpu_sys.GetNumMeshes() << " meshes" << std::endl;

// Create rigid ball_body simulation
ChSystemSMC sys_cube;
sys_cube.SetContactForceModel(ChSystemSMC::ContactForceModel::Hooke);
sys_cube.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
sys_cube.Set_G_acc(ChVector<>(0, 0, -980));
std::shared_ptr<ChBody>cube(sys_cube.NewBody());
cube->SetMass(500);
//130
cube->SetPos(ChVector<>(0, 0,-30));
//stator3->SetRot(Q_from_AngAxis(CH_C_PI_2, VECT_X));
cube->SetBodyFixed(false);
//auto sph = chrono_types::make_shared<ChSphereShape>();
//sph->GetSphereGeometry().rad = ;
//cube->AddAsset(sph);
sys_cube.AddBody(cube);

ChGpuVisualization gpu_vis(&gpu_sys, &sys_cube);
if (render) {
    gpu_vis.SetTitle("Chrono::Gpu ball cosim demo");
    gpu_vis.SetCameraPosition(ChVector<>(0, -200, 100), ChVector<>(0, 0, 0));
    gpu_vis.SetCameraMoveScale(1.0f);
    gpu_vis.Initialize();
}

std::string out_dir = GetChronoOutputPath() + "GPU/";
filesystem::create_directory(filesystem::path(out_dir));
out_dir = out_dir + params.output_dir;
filesystem::create_directory(filesystem::path(out_dir));

float iteration_step = params.step_size;
std::cout << "Output at    " << out_fps << " FPS" << std::endl;
std::cout << "Rendering at " << render_fps << " FPS" << std::endl;
unsigned int out_steps = (unsigned int)(1 / (out_fps * iteration_step));
unsigned int render_steps = (unsigned int)(1 / (render_fps * iteration_step));
unsigned int total_frames = (unsigned int)(0.33* out_fps);

int currframe = 0;
int currframee = 0;
unsigned int curr_step = 0;



clock_t start = std::clock();
// Output directory

std::string checkpoint_filep = out_dir + "/checkpointp.dat";


for (double t = 0; t <= 0.33; t += iteration_step, curr_step++) {




    gpu_sys.ApplyMeshMotion(0, cube->GetPos(), cube->GetRot(), cube->GetPos_dt(),
        cube->GetWvel_par());



    // the above function ses the current velocity and position, so you need to set it const each loop. Otherwise, it wil
    //follow the previuos step
    // // no rotation
    //cylinder_body->SetRot(ChQuaternion <>(1,0,0,0) );
    //stator->SetRot(ChMatrix33 <>(0, ChVector < >(0, 0, 1)));
    //cylinder_body->SetPos(ChVector <> (0,0,-t*200));
    //cylinder_body->SetPos_dt(ChVector<>(0, 0, -20));


    ChVector<double> cylinder_force;
    ChVector<> cylinder_torque;

    cube->Empty_forces_accumulators();
    cube->Accumulate_force(cylinder_force, cube->GetPos(), false);
    cube->Accumulate_torque(cylinder_torque, false);
    gpu_sys.CollectMeshContactForces(0, cylinder_force, cylinder_torque);






    if (curr_step % out_steps == 0) {

        std::cout << "Output frame " << currframe + 1 << " of " << total_frames << std::endl;
        char filename[300];
        char mesh_filename[300];
        sprintf(filename, "%s/stepp%06d.csv", out_dir.c_str(), currframe);
        sprintf(mesh_filename, "%s/stepp%06d_mesh", out_dir.c_str(), currframe++);
        gpu_sys.WriteParticleFile(std::string(filename));
        gpu_sys.WriteMeshes(std::string(mesh_filename));

        std::cout << cylinder_force << endl;
        

        char filename2[300];
        sprintf(filename2, "%s/contact%06d.csv", out_dir.c_str(), currframee++);
        gpu_sys.WriteContactInfoFile(std::string(filename2));

      

    }






if (render && curr_step % render_steps == 0) {
    if (gpu_vis.Render())

        break;
}




gpu_sys.AdvanceSimulation(iteration_step);
sys_cube.DoStepDynamics(iteration_step);

}

clock_t end = std::clock();
double total_time = ((double)(end - start)) / CLOCKS_PER_SEC;

gpu_sys.WriteCheckpointFile(checkpoint_filep);
std::cout << "Time: " << total_time << " seconds" << std::endl;
}

void runBallDrop(ChSystemGpuMesh& gpu_sys, ChGpuSimulationParameters& params) {

    // Add a cylinder mesh to the GPU system
    // approxiamte dimentions of CPT cone:
    float cylinder_radius = 12.5;
    float sphere_radius = 4;


    // density of AL 2.5 times sand:
    float cylinder_density = 1;
   // float cylinder_mass = (float)(CH_C_PI * 12.5 * 12.5 * 200 * cylinder_density)*0.8;
    float cylinder_mass = 500;
    float sphere_mass = (float)(CH_C_PI * sphere_radius * sphere_radius * (4 / 3) * params.sphere_density);
    gpu_sys.AddMesh(GetChronoDataFile("models/yong-cm2.obj"), ChVector<float>(0), ChMatrix33<float>(1),
        cylinder_mass);
    gpu_sys.AddMesh(GetChronoDataFile("models/sphere.obj"), ChVector<float>(0), ChMatrix33<float>(4),
        cylinder_mass);
    //gpu_sys.AddMesh(GetChronoDataFile("models/sphere.obj"), ChVector<float>(0), ChMatrix33<float>(sphere_radius),
    //    sphere_mass);


    // One more thing: we need to manually enable mesh in this run, because we disabled it in the settling phase,
    // let's overload that option.
    gpu_sys.EnableMeshCollision(true);

    gpu_sys.Initialize();
    std::cout << gpu_sys.GetNumMeshes() << " meshes" << std::endl;

    // Create rigid ball_body simulation
    ChSystemSMC sys_cylinder;
    sys_cylinder.SetContactForceModel(ChSystemSMC::ContactForceModel::Hooke);
    sys_cylinder.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
    sys_cylinder.Set_G_acc(ChVector<>(0, 0,-980));

    double inertia = 1.0 / 2.0 * cylinder_mass * cylinder_radius * cylinder_radius;
    double inertia2 = 1.0 / 12.0 * cylinder_mass * ((3 * cylinder_radius * cylinder_radius) +20 * 20);
    //103
    ChVector<> cylinder_initial_pos(0, 0, (-129));


    //std::shared_ptr<ChBody>stator2(sys_cylinder.NewBody());
    //stator2->SetMass(0);
    ////145
    //stator2->SetPos(ChVector<>(0, 0, 145));
    ////stator3->SetRot(Q_from_AngAxis(CH_C_PI_2, VECT_X));
    //stator2->SetBodyFixed(true);
    //sys_cylinder.AddBody(stator2);


    std::shared_ptr<ChBody>stator(sys_cylinder.NewBody());
    stator->SetMass(cylinder_mass);
    //130
    stator->SetPos(ChVector<>(0, 0, 90));
    //stator3->SetRot(Q_from_AngAxis(CH_C_PI_2, VECT_X));
    stator->SetBodyFixed(false);
    //stator->SetInertiaXX(ChVector<>(inertia2, inertia2, inertia));
    sys_cylinder.AddBody(stator);




    std::shared_ptr<ChBody> cylinder_body(sys_cylinder.NewBody());
    cylinder_body->SetMass(cylinder_mass);
    cylinder_body->SetInertiaXX(ChVector<>(inertia2, inertia2, inertia));
    cylinder_body->SetPos(cylinder_initial_pos);
    cylinder_body->SetBodyFixed(false);
    //cylinder_body->SetPos_dt( ChVector<>(0, 0, -200));
    sys_cylinder.AddBody(cylinder_body);

    //create motor and stator:




   // auto rotmotor2 = chrono_types::make_shared<ChLinkMotorLinearSpeed>();
   // // Connect the rotor and the stator and add the motor to the system:
   //// ChFrame<> A(stator->GetPos(), Q_from_AngX(0));
   // rotmotor2->Initialize(stator,                // body A (slave)
   //     stator2,               // body B (master)
   //     ChFrame<>(ChVector<>(0, 0, 145), Q_from_AngY(CH_C_PI/2))); // motor frame, in abs. coords
   // //rotmotor1->SetSpindleConstraint(true, false, true, true, true);
   // sys_cylinder.Add(rotmotor2);

   // // Create a ChFunction to be used for the ChLinkMotorRotationSpeed
   // auto mwspeed2 = chrono_types::make_shared<ChFunction_Const>(30);
   //   
   // rotmotor2->SetSpeedFunction(mwspeed2);


    /*
    // Create the motor for augur
    auto rotmotor1 = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    // Connect the rotor and the stator and add the motor to the system:
   // ChFrame<> A(stator->GetPos(), Q_from_AngX(0));
    rotmotor1->Initialize(cylinder_body,                // body A (slave)
        stator,               // body B (master)
        ChFrame<> (stator->GetPos(), Q_from_AngX(0))); // motor frame, in abs. coords
    //rotmotor1->SetSpindleConstraint(true, false, true, true, true);
    sys_cylinder.Add(rotmotor1);
   
    // Create a ChFunction to be used for the ChLinkMotorRotationSpeed
    auto mwspeed =
        chrono_types::make_shared<ChFunction_Const>(6);
    // constant angular speed, in [rad/s], 1PI/s =180°/s
    // Let the motor use this motion function:
    rotmotor1->SetSpeedFunction(mwspeed);

    */
   












    ChGpuVisualization gpu_vis(&gpu_sys, &sys_cylinder);
    if (render) {
        gpu_vis.SetTitle("Chrono::Gpu ball cosim demo");
        gpu_vis.SetCameraPosition(ChVector<>(0, -200, 100), ChVector<>(0, 0, 0));
        gpu_vis.SetCameraMoveScale(1.0f);
        gpu_vis.Initialize();
    }

    std::string out_dir = GetChronoOutputPath() + "GPU/";
    filesystem::create_directory(filesystem::path(out_dir));
    out_dir = out_dir + params.output_dir;
    filesystem::create_directory(filesystem::path(out_dir));

    float iteration_step = params.step_size;
    std::cout << "Output at    " << out_fps << " FPS" << std::endl;
    std::cout << "Rendering at " << render_fps << " FPS" << std::endl;
    unsigned int out_steps = (unsigned int)(1 / (out_fps * iteration_step));
    unsigned int render_steps = (unsigned int)(1 / (render_fps * iteration_step));
    unsigned int total_frames = (unsigned int)(params.time_end * out_fps);

    int currframe = 0;
    int currframee = 0;
    unsigned int curr_step = 0;
    

    
    clock_t start = std::clock();
  
    /////////////me
    //gpu_sys.EnableMeshCollision(true);
    gpu_sys.SetRecordingContactInfo(true);



////meshforceINFO:
//    /// Set sphere-to-mesh static friction coefficient.
//    gpu_sys.SetStaticFrictionCoeff_SPH2MESH(2.0);
//    /// Set sphere-to-mesh rolling friction coefficient.
//    gpu_sys.SetRollingCoeff_SPH2MESH(2.0);
//    /// Set sphere-to-mesh spinning friction coefficient.
//    gpu_sys.SetSpinningCoeff_SPH2MESH(2.0);
//
//    /// Set sphere-to-mesh normal contact stiffness.
//    gpu_sys.SetKn_SPH2MESH( 2.2);
//    /// Set sphere-to-mesh normal damping coefficient.
//    gpu_sys.SetGn_SPH2MESH(2.0);
//
//    /// Set sphere-to-mesh tangential contact stiffness.
//    gpu_sys.SetKt_SPH2MESH(2.0);
//    /// Set sphere-to-mesh tangential damping coefficient.
//    gpu_sys.SetGt_SPH2MESH(2.0);
//
//    /// Set the ratio of adhesion force to sphere weight for sphere to mesh.
//    gpu_sys.SetAdhesionRatio_SPH2MESH(2.0);

    

    int n = 0;
    int i = 0;
    ChQuaternion<double> rotme(1, 0, 0, 0);
    chrono::ChMatrixNM<double, 31, 3> myforce;
    chrono::ChMatrixNM<double, 31, 1> myforcex;
    chrono::ChMatrixNM<double, 31, 1> myforcey;
    chrono::ChMatrixNM<double, 31, 1> myforcez;
    chrono::ChMatrixNM<double, 31, 3> mypositionz;

   
    myforce.setZero();
    myforcex.setZero();
    myforcey.setZero();
    myforcez.setZero();
    mypositionz.setZero();


    //CSystemGpu::SetParticleFixed(true);

    for (double t = 0; t < (double)params.time_end; t += iteration_step, curr_step++) {

       // const speed since the augur will be confined to stator even though we have gravity
        //ChVector<double> V = cylinder_body->GetPos_dt();

        //stator->SetRot(ChMatrix33 <>(0, ChVector < >(0, 0, 1)));

        //gpu_sys.ApplyMeshMotion(2, stator2->GetPos(), stator2->GetRot(), stator2->GetPos_dt(),
        //    stator2->GetWvel_par());
      //  gpu_sys.ApplyMeshMotion(1, stator->GetPos(), stator->GetRot(), stator->GetPos_dt(),
          //stator->GetWvel_par());
       gpu_sys.ApplyMeshMotion(0, cylinder_body->GetPos(), cylinder_body->GetRot(), cylinder_body->GetPos_dt(),
            cylinder_body->GetWvel_par());
        //cylinder_body->SetPos_dt(ChVector<>(0, 0, (-0.5 * 980 * t * t)));
        cylinder_body->SetRot(ChMatrix33 <>((-t)*3, ChVector < >(0, 0, 1)));



        // the above function ses the current velocity and position, so you need to set it const each loop. Otherwise, it wil
        //follow the previuos step
        // // no rotation
        //cylinder_body->SetRot(ChQuaternion <>(1,0,0,0) );
        //stator->SetRot(ChMatrix33 <>(0, ChVector < >(0, 0, 1)));
        //cylinder_body->SetPos(ChVector <> (0,0,-t*200));
        //cylinder_body->SetPos_dt(ChVector<>(0, 0, -20));
       

        ChVector<double> cylinder_force;
        ChVector<> cylinder_torque;
        gpu_sys.CollectMeshContactForces(0, cylinder_force, cylinder_torque);
        cylinder_body->Empty_forces_accumulators();
       // stator->Empty_forces_accumulators();
        cylinder_body->Accumulate_force(cylinder_force, cylinder_body->GetPos(), false);
        cylinder_body->Accumulate_torque(cylinder_torque, false);
       // stator->Accumulate_force(cylinder_force, stator->GetPos(), false);
        //stator->Accumulate_torque(cylinder_torque, false);


        n++;
        

        if (curr_step % out_steps == 0) {

            std::cout << "Output frame " << currframe + 1 << " of " << total_frames << std::endl;
            char filename[300];
            char mesh_filename[300];
            sprintf(filename, "%s/steprotN%06d.csv", out_dir.c_str(), currframe);
            sprintf(mesh_filename, "%s/steprotN%06d_mesh", out_dir.c_str(), currframe++);
            gpu_sys.WriteParticleFile(std::string(filename));
            gpu_sys.WriteMeshes(std::string(mesh_filename));
           
            std::cout << "cylinder_force" << endl;
            std::cout << cylinder_body->GetPos()[2] << endl;
            std::cout << stator->GetPos()[2]  << endl;
            std::cout << stator->GetPos()[2]- cylinder_body->GetPos()[2] << endl;
            std::cout << cylinder_force << endl;
            std::cout << cylinder_force[0] << endl;
            std::cout << cylinder_force[1] << endl;
            std::cout << cylinder_force[2] << endl;
            //one matrix
            myforce((i * 3)) = cylinder_force[0];
            myforce((i * 3) + 1) = cylinder_force[1];
            myforce((i * 3) + 2) = cylinder_force[2];
            i++;
//seperate columns
            myforcex(i) = cylinder_force[0];
            myforcey(i) = cylinder_force[1];
            myforcez(i) = cylinder_force[2];
            mypositionz(i) = cylinder_body->GetPos()[2];

           // myforce(n, 2) = cylinder_force[2];
            //myforce(n, 3) = cylinder_force[3];
           
            std::cout << myforce << endl;
            //std::cout << cylinder_body->GetRot() << endl;
    
            char filename2[300];
            sprintf(filename2, "%s/contact%06d.csv", out_dir.c_str(), currframee++);
           gpu_sys.WriteContactInfoFile(std::string(filename2));
        
           n++;
           //VoidRatio(162978, 175, params);
          
        }

        

        if (render && curr_step % render_steps == 0) {
            if (gpu_vis.Render())

                break;
        }

     


        gpu_sys.AdvanceSimulation(iteration_step);
        sys_cylinder.DoStepDynamics(iteration_step);
        
    }
    
    std::cout << "myforcex: " << endl;
    std::cout << myforcex << endl;
    std::cout << "myforcey: " << endl;
    std::cout << myforcey << endl;
    std::cout << "myforcez: " << endl;
    std::cout << myforcez << endl;
    std::cout << "mypositionz: " << endl;
    std::cout << mypositionz << endl;


    /////////////////////output to file:
#include <iostream>
    using std::cerr;
    using std::endl;
#include <fstream>
    using std::ofstream;
#include <cstdlib> // for exit function
    // This program output values from an array to a file named example2.dat


    ofstream outdata; // outdata is like cin
    int j; // loop index
    //int num[5] = { 4, 3, 6, 7, 12 }; // list of output values

    outdata.open("myforcezrotN75.dat"); // opens the file
    if (!outdata) { // file couldn't be opened
        cerr << "Error: file could not be opened" << endl;
        exit(1);
    }

    for (j = 0; j < 62; ++j)
        outdata << myforcez(j) << endl;
    outdata.close();

    outdata.open("myforcexrotN75.dat"); // opens the file
    if (!outdata) { // file couldn't be opened
        cerr << "Error: file could not be opened" << endl;
        exit(1);
    }

    for (j = 0; j < 62; ++j)
        outdata << myforcex(j) << endl;
    outdata.close();
    outdata.open("myforceyrotN75.dat"); // opens the file
    if (!outdata) { // file couldn't be opened
        cerr << "Error: file could not be opened" << endl;
        exit(1);
    }

    for (j = 0; j < 62; ++j)
        outdata << myforcey(j) << endl;
    outdata.close();
    outdata.open("mypositionzrot.dat"); // opens the file
    if (!outdata) { // file couldn't be opened
        cerr << "Error: file could not be opened" << endl;
        exit(1);
    }

    for (j = 0; j < 62; ++j)
        outdata << mypositionz(j) << endl;
    outdata.close();




    //////////////////////////////////////////////////////////


    clock_t end = std::clock();
    double total_time = ((double)(end - start)) / CLOCKS_PER_SEC;
   

    std::cout << "Time: " << total_time << " seconds" << std::endl;
}

int main(int argc, char* argv[]) {
    ChGpuSimulationParameters params;
    if (argc != 2 || ParseJSON(gpu::GetDataFile(argv[1]), params) == false) {
        std::cout << "Usage:\n./demo_GPU_ballcosim <json_file>" << std::endl;
        return 1;
    }

    if (params.run_mode > 3) {
        printf("ERROR! Unknown run_mode specified!\n");
        return 2;
    }

    // Output directory
    std::string out_dir = GetChronoOutputPath() + "GPU/";
    filesystem::create_directory(filesystem::path(out_dir));
    out_dir = out_dir + params.output_dir;
    filesystem::create_directory(filesystem::path(out_dir));

    std::string checkpoint_file = out_dir + "/checkpoint.dat";
    std::string checkpoint_filep = out_dir + "/checkpointp.dat";

  

    if (params.run_mode == 1) {
        // run_mode = 1, this is a restarted run

        // Load checkpoint file.
        // Note that with current version, user defined meshes and boundaries are not stored in the checkpoint file,
        // so they must be manually set later. This behavior will be improved in later patches.
        // Simulation parameters and particle states are all in with this file loaded.
        ChSystemGpuMesh gpu_sys(checkpoint_file);

        // Add a ball through a mesh, whose dynamics are managed by Chrono Core, and run this co-simulation.
        runBallDrop(gpu_sys, params);
        //now exits main funtion!!:
        std::string force_file = out_dir + "/force.dat";


        return 0;
    }
    if (params.run_mode == 2) {

        ChSystemGpuMesh gpu_sys(checkpoint_file);

        cube(gpu_sys, params);

        std::string force_file = out_dir + "/force.dat";


        return 0;
    }


    // run_mode = 0, this is a newly started run. We have to set all simulation params.
    ChSystemGpuMesh gpu_sys(params.sphere_radius, params.sphere_density,
        ChVector<float>(params.box_X, params.box_Y, params.box_Z));

    printf(
        "Now run_mode == 0, this run is particle settling phase.\n"
        "After it is done, you will have a settled bed of granular material.\n"
        "A checkpoint file will be generated in the output directory to store this state.\n"
        "You can then open the JSON file, change \"run_mode\" from 0 to 1, then run this demo again,\n"
        "to proceed with the ball drop part of this demo.\n\n");

    float iteration_step = params.step_size;

    // z box is the from bottom to cylinder and fill top is where at the top particles begin till to fill bottom
    double fill_bottom = -params.box_Z / 2.0;
    double fill_top = params.box_Z / 4.0;

    chrono::utils::PDSampler<float> sampler(2.4f * params.sphere_radius);
    // chrono::utils::HCPSampler<float> sampler(2.05 * params.sphere_radius);

    // leave a 4cm margin at edges of sampling
    ChVector<> hdims(params.box_X / 2 - 4.0, params.box_Y / 2 - 4.0, 0);
    ChVector<> center(0, 0, fill_bottom + 2.0 * params.sphere_radius);
    std::vector<ChVector<float>> body_points;

    // Shift up for bottom of box
    center.z() += 3 * params.sphere_radius;
    while (center.z() < fill_top) {
        // You can uncomment this line to see a report on particle creation process.
        std::cout << "Create layer at " << center.z() << std::endl;
        auto points = sampler.SampleBox(center, hdims);
        body_points.insert(body_points.end(), points.begin(), points.end());
        center.z() += 2.05 * params.sphere_radius;
    }
    std::cout << body_points.size() << " particles sampled!" << std::endl;





    gpu_sys.SetParticles(body_points);

    gpu_sys.SetKn_SPH2SPH(params.normalStiffS2S);
    gpu_sys.SetKn_SPH2WALL(params.normalStiffS2W);
    gpu_sys.SetKn_SPH2MESH(params.normalStiffS2M);

    gpu_sys.SetGn_SPH2SPH(params.normalDampS2S);
    gpu_sys.SetGn_SPH2WALL(params.normalDampS2W);
    gpu_sys.SetGn_SPH2MESH(params.normalDampS2M);

    gpu_sys.SetKt_SPH2SPH(params.tangentStiffS2S);
    gpu_sys.SetKt_SPH2WALL(params.tangentStiffS2W);
    gpu_sys.SetKt_SPH2MESH(params.tangentStiffS2M);

    gpu_sys.SetGt_SPH2SPH(params.tangentDampS2S);
    gpu_sys.SetGt_SPH2WALL(params.tangentDampS2W);
    gpu_sys.SetGt_SPH2MESH(params.tangentDampS2M);

    gpu_sys.SetCohesionRatio(params.cohesion_ratio);
    gpu_sys.SetAdhesionRatio_SPH2MESH(params.adhesion_ratio_s2m);
    gpu_sys.SetAdhesionRatio_SPH2WALL(params.adhesion_ratio_s2w);

    gpu_sys.SetGravitationalAcceleration(ChVector<float>(params.grav_X, params.grav_Y, params.grav_Z));

    gpu_sys.SetFixedStepSize(params.step_size);
    gpu_sys.SetFrictionMode(CHGPU_FRICTION_MODE::MULTI_STEP);
    gpu_sys.SetTimeIntegrator(CHGPU_TIME_INTEGRATOR::CENTERED_DIFFERENCE);

    gpu_sys.SetStaticFrictionCoeff_SPH2SPH(params.static_friction_coeffS2S);
    gpu_sys.SetStaticFrictionCoeff_SPH2WALL(params.static_friction_coeffS2W);
    gpu_sys.SetStaticFrictionCoeff_SPH2MESH(params.static_friction_coeffS2M);

    // gpu_sys.SetRollingCoeff_SPH2SPH(params.rolling_friction_coeffS2S);
    // gpu_sys.SetRollingCoeff_SPH2WALL(params.rolling_friction_coeffS2W);
    // gpu_sys.SetRollingCoeff_SPH2MESH(params.rolling_friction_coeffS2M);

    gpu_sys.SetParticleOutputMode(params.write_mode);
    gpu_sys.SetVerbosity(params.verbose);
    gpu_sys.SetBDFixed(true);

    // In the settling run we disable the mesh.
    gpu_sys.EnableMeshCollision(false);


    //meee
    gpu_sys.SetRecordingContactInfo(true);
    /*
    // We could prescribe the motion of the big box domain. But here in this demo we will not do that.
    std::function<double3(float)> pos_func_wave = [&params](float t) {
        double3 pos = {0, 0, 0};

        double t0 = 0.5;
        double freq = CH_C_PI / 4;

        if (t > t0) {
            pos.x = 0.1 * params.box_X * std::sin((t - t0) * freq);
        }
        return pos;
    };

    gpu_sys.setBDWallsMotionFunction(pos_func_wave);
    */

    gpu_sys.Initialize();

    unsigned int out_steps = (unsigned int)(1 / (out_fps * iteration_step));
    unsigned int total_frames = (unsigned int)(params.time_end * out_fps);
    int currframe = 0;
    int currframee = 0;
    unsigned int curr_step = 0;
    for (double t = 0; t < (double)params.time_end; t += iteration_step, curr_step++) {
        if (curr_step % out_steps == 0) {
            std::cout << "Output frame " << currframe + 1 << " of " << total_frames << std::endl;
            char filename[100];

            sprintf(filename, "%s/stepp%06d.csv", out_dir.c_str(), currframe++);
            gpu_sys.WriteParticleFile(std::string(filename));



        }


        gpu_sys.AdvanceSimulation(iteration_step);
    }









    // out_steps = (unsigned int)(1 / (out_fps * iteration_step));
     //total_frames = (unsigned int)(params.time_end * out_fps);
     //currframee = 0;
     //curr_step = 0;

     //for (double t = 0; t < (double)params.time_end; t += iteration_step, curr_step++) {
         //if (curr_step % out_steps == 0) {
            // char filename2[100];
             //sprintf(filename2, "%s/contact%06d.csv", out_dir.c_str(), currframee++);
             //gpu_sys.WriteContactInfoFile(std::string(filename2));
         //}
    // }

     // This is settling phase, so output a checkpoint file








    gpu_sys.WriteCheckpointFile(checkpoint_file);

    return 0;
}
