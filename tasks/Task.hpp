/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef JOINT_DRIVER_TASK_TASK_HPP
#define JOINT_DRIVER_TASK_TASK_HPP

#include <RpcIfClient.h> /** This is the RPC intreface for 3DROV **/
#include <Eigen/Geometry> /** Eigen data type for Matrix, Quaternion, etc... */

/** Rock libraries **/
#include "frame_helper/FrameHelper.h" /** Rock lib for manipulate frames **/

#include "lowlevel_rpc_3drov/TaskBase.hpp"


namespace lowlevel_rpc_3drov {

    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','lowlevel_rpc_3drov::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:

        static const int NUMBER_OF_WHEELS = 4;
        static const double R2D = 180.00/M_PI; /** Convert radian to degree **/
        static const double D2R = M_PI/180.00; /** Convert degree to radian **/

        RpcIfClientNamespace::RpcIfClient *rpcClient;
        /**
         *  * Post a command, wait until it is executed and return the result.
         *   * This is a blocking function, it will stall the calling thread
         *    * until the simulation step is executed (several ms)
         *     */  
        char* execute(std::string cmd);

        /**
         *  * Register a command that will be executed automatically after
         *   * each simulation step. The result can be retrieved by calling
         *    * the getLastResult() function.
         *     */
         bool registerCommand(std::string cmd);

        /**
         *  * Unregister a command. It will no longer executed automatically
         *   * after each simulation step
         *    */
        bool unregisterCommand(std::string cmd);

        /**
         *  * Retrieve the results of the execution of a registered command after
         *   * the last simulation step. 
         *    */
        char* getLastResult(std::string cmd);

        /** 3DROV buffers variables **/
        char buffer_position[2048];
        char buffer_velocity[2048];
        char buffer_acceleration[2048];
        char buffer_inclino[2048];
        char buffer_rearaxis[2048];
        char buffer_frontaxis[2048];


        /** Port variables class **/
        base::actuators::Status encodersSamples; //Encoders values of the joints
        base::samples::IMUSensors imuSamples;//imu samples
        base::samples::RigidBodyState poseOut;// Body Center w.r.t the World Coordinate system
        RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> frame_out;//Images from the Left NavCam
        frame_helper::FrameHelper frameHelper; //Frame helper to load the image comming from the RPC call

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "lowlevel_rpc_3drov::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();



    };
}

#endif

