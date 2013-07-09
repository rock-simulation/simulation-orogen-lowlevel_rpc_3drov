#! /usr/bin/env ruby

require 'orocos'
require 'orocos/log'
require 'transformer'
require 'vizkit'
require 'utilrb'
require 'eigen'

include Orocos
Orocos::CORBA.max_message_size = 90000000000000


#Initializes the CORBA communication layer
Orocos.initialize

Orocos.run 'joint_driver::Task' => 'joint' do
	# get the task
    joint_task = Orocos.name_service.get 'joint'

    joint_task.configure
    joint_task.start
    while 1==1
        
    end

end


