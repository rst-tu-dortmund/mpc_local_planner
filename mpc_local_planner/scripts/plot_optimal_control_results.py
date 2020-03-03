#!/usr/bin/env python
#
#
#  Software License Agreement
#
#  Copyright (c) 2020, Christoph Roesmann, All rights reserved.
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
#  Authors: Christoph Roesmann

import rospy, math
from mpc_local_planner_msgs.msg import OptimalControlResult
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from threading import Lock

class OcpResultPlotter:

  def __init__(self, plot_states, topic_name):

    self.initialized = False
    self.plot_states = plot_states
    self.dim_states = 0
    self.dim_controls = 0
    self.x_fig = plt.Figure()
    self.x_axes = []
    self.u_fig = plt.Figure()
    self.u_axes = []
    self.tx = []
    self.x = []
    self.tu = []
    self.u = []
    self.mutex = Lock()
    
    self.sub = rospy.Subscriber(topic_name, OptimalControlResult, self.ocp_result_callback, queue_size = 1)
    rospy.loginfo("Plotting OCP results published on '%s'.",topic_name) 
    rospy.loginfo("Make sure to enable rosparam 'controller/publish_ocp_results' in the mpc_local_planner.") 

  def ocp_result_callback(self, data):
    rospy.loginfo_once("First message received.")
    if not self.initialized:
      self.dim_states = data.dim_states
      self.dim_controls = data.dim_controls
  
    # Read data
    self.mutex.acquire()
    if self.plot_states:
      self.tx = np.array(data.time_states)
      self.x = np.matrix(data.states)
      self.x = np.reshape(self.x, (self.tx.size, int(self.dim_states)))
    self.tu = np.array(data.time_controls)
    self.u = np.matrix(data.controls)
    self.u = np.reshape(self.u, (self.tu.size, int(self.dim_controls)))
    self.mutex.release()
    
  def initializePlotWindows(self):
    if self.plot_states:
      self.x_fig, self.x_axes = plt.subplots(self.dim_states, sharex=True)
      self.x_axes[0].set_title('States')
      for idx, ax in enumerate(self.x_axes):
        ax.set_ylabel("x" + str(idx))
      self.x_axes[-1].set_xlabel('Time [s]')

    self.u_fig, self.u_axes = plt.subplots(self.dim_controls, sharex=True)
    self.u_axes[0].set_title('Controls')
    for idx, ax in enumerate(self.u_axes):
      ax.set_ylabel("u" + str(idx))
    self.u_axes[-1].set_xlabel('Time [s]')
    plt.ion()
    plt.show()

  
  def plot(self):
    # We recreate the plot every time, not fast, but okay for now....
    self.mutex.acquire()
    if self.plot_states:
      for idx, ax in enumerate(self.x_axes):
        ax.cla()
        ax.grid()
        ax.plot(self.tx, self.x[:,idx])
        ax.get_yaxis().set_major_formatter(ticker.FuncFormatter(lambda x, p: "%.2f" % x))
        ax.set_ylabel("x" + str(idx))
      self.x_axes[0].set_title('States')
      self.x_axes[-1].set_xlabel('Time [s]')
      self.x_fig.canvas.draw()

    for idx, ax in enumerate(self.u_axes):
      ax.cla()
      ax.grid()
      ax.step(self.tu, self.u[:, idx], where='post')
      ax.get_yaxis().set_major_formatter(ticker.FuncFormatter(lambda x, p: "%.2f" % x))
      ax.set_ylabel("u" + str(idx))
    self.u_axes[0].set_title('Controls')
    self.u_axes[-1].set_xlabel('Time [s]')
    self.u_fig.canvas.draw()
    self.mutex.release()
    
  def start(self, rate):
    r = rospy.Rate(rate) # define rate here
    while not rospy.is_shutdown():
      if not self.initialized and (self.dim_states > 0 or self.dim_controls > 0):
        self.initializePlotWindows()
        self.initialized = True
      if self.initialized:
        self.plot()
      r.sleep()



if __name__ == '__main__': 
  try:
   
    rospy.init_node("ocp_result_plotter", anonymous=True) 

    topic_name = "/test_mpc_optim_node/ocp_result"
    topic_name = rospy.get_param('~ocp_result_topic', topic_name)

    plot_states = rospy.get_param('~plot_states', False)

    result_plotter = OcpResultPlotter(plot_states, topic_name)
    rate = 2
    topic_name = rospy.get_param('~plot_rate', rate)
    result_plotter.start(rate)
  except rospy.ROSInterruptException:
    pass
