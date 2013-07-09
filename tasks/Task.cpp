/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace lowlevel_rpc_3drov;
using namespace RpcIfClientNamespace;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    rpcClient = NULL;
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
    rpcClient = NULL;
}

Task::~Task()
{
}


char* Task::execute(std::string cmd) {
 char* result = rpcClient->orc_rpc_executeCommand((char*) cmd.c_str());

if (result != NULL) {
    printf("%s\n", result);
      }
return result;
}

/**
 *  * Register a command that will be executed automatically after
 *   * each simulation step. The result can be retrieved by calling
 *    * the getLastResult() function.
 *     */
bool Task::registerCommand(std::string cmd) {
      bool res = rpcClient->orc_rpc_registerCommand((char*) cmd.c_str());
        return res;
}

/**
 *  * Unregister a command. It will no longer executed automatically
 *   * after each simulation step
 *    */
bool Task::unregisterCommand(std::string cmd) {
      bool res = rpcClient->orc_rpc_unregisterCommand((char*) cmd.c_str());
        return res;
}

/**
 *  * Retrieve the results of the execution of a registered command after
 *   * the last simulation step. 
 *    */
char* Task::getLastResult(std::string cmd) {
      char* result = rpcClient->orc_rpc_getCommandLastResult((char*) cmd.c_str());

        if (result != NULL) {
                printf("%s\n", result);
                  }
          return result;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    if (rpcClient == NULL)
        rpcClient = new RpcIfClient("localhost", RpcIfClient::RSMAPP_DYNAMIC_MODEL);


    /** Init the output ports variables **/
    encodersSamples.resize(Task::NUMBER_OF_WHEELS);
    poseOut.invalidate();

    ::base::samples::frame::Frame *oframe = new ::base::samples::frame::Frame();

    frame_out.reset(oframe);
    oframe = NULL;


    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    // do not calculate dynamics automatically
    execute("setManualStepDynamics;true\n");

    // Set wheels speed (in (-1) rad/sec)
    execute("setDynamicMotorVelocity;terrainCell;ratRoverVFS;wheelFL;-1.0,setDynamicMotorVelocity;terrainCell;ratRoverVFS;wheelFR;-1.0,setDynamicMotorVelocity;terrainCell;ratRoverVFS;wheelRL;-1.0,setDynamicMotorVelocity;terrainCell;ratRoverVFS;wheelRR;-1.0\n");

    strcpy(buffer_position,"getActorPose;terrainCell;ratRoverVFS;baseBody\n");
    sprintf(buffer_velocity,"getActorVelocity;terrainCell;ratRoverVFS;baseBody;local\n");
    sprintf(buffer_acceleration,"getActorAcceleration;terrainCell;ratRoverVFS;baseBody;local\n");
    sprintf(buffer_inclino,"getInclinoData;terrainCellratRoverVFSincl01\n");

    registerCommand(buffer_position);
    registerCommand(buffer_velocity);
    registerCommand(buffer_acceleration);
    registerCommand(buffer_inclino);

    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();


    /** Get position from rover **/
    double position[6];
    char* result_position = getLastResult(buffer_position);
    /** Time stamps **/
    poseOut.time = base::Time::now();
    imuSamples.time = poseOut.time;
    encodersSamples.time = imuSamples.time;

    if (result_position != NULL)
    {
        sscanf(result_position, "%lf,%lf,%lf,%lf,%lf,%lf",
        &position[0],
        &position[1],
        &position[2],
        &position[3],
        &position[4],
        &position[5]);
    }

    /** Fill the rock struct **/
    poseOut.position[0] = position[0];
    poseOut.position[1] = position[1];
    poseOut.position[2] = position[2];

    poseOut.orientation = Eigen::Quaternion <double> (
                        Eigen::AngleAxisd(position[5] * Task::D2R, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(position[4] * Task::D2R, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(position[3] * Task::D2R, Eigen::Vector3d::UnitX()));

    /** Get rover velocity **/
    double roverVelocity[6];
    char* result_vel = getLastResult(buffer_velocity);
    if (result_vel != NULL)
    {
        sscanf(result_vel, "%lf,%lf,%lf,%lf,%lf,%lf",
        &roverVelocity[0],
        &roverVelocity[1],
        &roverVelocity[2],
        &roverVelocity[3],
        &roverVelocity[4],
        &roverVelocity[5]);
    }

    /** Fill the rock struct **/
    poseOut.velocity[0] = roverVelocity[0];
    poseOut.velocity[1] = roverVelocity[1];
    poseOut.velocity[2] = roverVelocity[2];

    /** Fill the rock struct for the IMU **/
    imuSamples.gyro[0] = roverVelocity[3];
    imuSamples.gyro[1] = roverVelocity[4];
    imuSamples.gyro[2] = roverVelocity[5];

    /** Get rover acceleration **/
    double roverAcceleration[6];
    char* result_acc = getLastResult(buffer_acceleration);
    if (result_acc != NULL)
    {
        sscanf(result_acc, "%lf,%lf,%lf,%lf,%lf,%lf",
        &roverAcceleration[0],
        &roverAcceleration[1],
        &roverAcceleration[2],
        &roverAcceleration[3],
        &roverAcceleration[4],
        &roverAcceleration[5]);
    }

    /** Fill the rock struct for the IMU **/
    imuSamples.acc[0] = roverAcceleration[0];
    imuSamples.acc[1] = roverAcceleration[1];
    imuSamples.acc[2] = roverAcceleration[2];

    // Generate an image
    execute("renderImageToFile;terrainCellratRoverVFSNavCamLeft;/home/javi/left_frame;.png\n");

    ::base::samples::frame::Frame *frame_ptr = frame_out.write_access();
    frameHelper.loadFrame ("/home/javi/left_frame.png", *frame_ptr);
    std::cout<<"image size:"<< frame_ptr->size.width<<" by "<<frame_ptr->size.height<<"\n";
    std::cout<<"data depth:"<< frame_ptr->data_depth<<" frame_mode:"<<frame_ptr->frame_mode<<"\n";

    frame_ptr->time = poseOut.time;
    frame_out.reset(frame_ptr);

    /* Run the  step */
    execute("stepDynamics\n");


    /** Write in the RTT ports **/
    _pose_samples_out.write(poseOut);
    _inertial_samples_out.write(imuSamples);
    _encoder_samples_out.write(encodersSamples);
    _frame_samples_out.write(frame_out);

}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();

   // Set wheels speed (in (-1) rad/sec)
   execute("setDynamicMotorVelocity;terrainCell;ratRoverVFS;wheelFL;-0.0,setDynamicMotorVelocity;terrainCell;ratRoverVFS;wheelFR;-0.0,setDynamicMotorVelocity;terrainCell;ratRoverVFS;wheelRL;-0.0,setDynamicMotorVelocity;terrainCell;ratRoverVFS;wheelRR;-0.0\n");

}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
