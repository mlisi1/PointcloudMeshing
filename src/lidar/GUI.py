#=====================================================================================================#
#                                        		GUI                                                   #
#=====================================================================================================#
# 
# Library that handles the GUI for the streamer node. Mostly thought to add checkboxes and sliders
# for filters.
# In general, it can add sliders, checkboxes, spacers and buttons dynamically resizing the window.
# It also supports interaction between checkboxes and sliders: if the checkbox is set to false, the
# relative sliders will be disabled.
#
#------------------------------------------------------------------------------------------------------
from tkinter import *
import rospy


#Main class; it handles the window cration
class GUI:

	#Initialize the window and two dicts: one for variable correspondence and one for element interactions
	def __init__(self, window_name):

		self.root = Tk()
		self.root.geometry('140x100')
		self.root.resizable(FALSE, FALSE)
		self.height = 0
		self.root.title(window_name)
		self.GUI_instances = 0
		self.dict = {'int':IntVar, 'string':StringVar, 'float':DoubleVar}
		self.interactions = {}

	#Element creation functions
	def create_slider(self, name, var_type, from_, to, increment, def_val = None):

		return GUI.Slider(self, name, var_type, from_, to, increment, def_val)

	def add_play_button(self):

		return GUI.PlayButton(self)

	def add_spacer(self, empty = False):

		return GUI.Spacer(self, empty)

	def add_checkbutton(self, label, on = True):

		return GUI.CheckButton(self, label, on)

	#Function that updates the geometry of the window; it gets called in every subclass __init__()
	def update_geometry(self, offset = 70):

		height = self.GUI_instances * offset
		self.height += offset
		self.root.geometry('160x'+str(self.height))

	#Simple function to enable/disable sliders given the checkboxes values
	def handle_interactions(self):

		if len(self.interactions) > 0:

			for key in self.interactions:

				values = self.interactions[key]

				for value in values:

					if key.var.get():

						value.scale.config(state = NORMAL, sliderrelief = RAISED)

					else:

						value.scale.config(state = DISABLED, sliderrelief = FLAT)


	#Updates GUI if ROS is running
	def tk_routine(self):



		if not rospy.is_shutdown():

			self.handle_interactions()
			self.root.update_idletasks()
			self.root.update()

		else:

			exit()

	#Slider class definition; 
	class Slider:

		def __init__(self, parent,  name, var_type, from_, to, increment, def_val):	

			self.var = parent.dict[var_type]()
			parent.GUI_instances += 1
			self.scale = Scale(parent.root, 
				variable = self.var, 
				resolution = increment, 
				from_ = from_, 
				to = to,
				orient = HORIZONTAL,
				label = name)
			

			if def_val is not None:
				self.var.set(def_val)
				
			self.scale.pack()

			parent.update_geometry(offset = 62)

	#PlayButton class definition
	class PlayButton:

		def __init__(self, parent):

			self.button = Button(parent.root, 
				command = self.press,
				text = "Pause",)
			parent.GUI_instances +=1
			self.play = True
			self.button.pack()

			parent.update_geometry(offset = 25)

		#On press, it updates it label and its play value
		def press(self):

			if self.play:

				self.button.config(text = 'Play')

			else:

				self.button.config(text = 'Pause')

			self.play = not self.play

	#Spacer class definition; it just leaves blank space
	class Spacer:

		def __init__(self, parent, empty):

			text = "___________________"

			if empty:

				text = ""

			self.spacer = Label(parent.root, text = text)
			self.spacer.pack()
			parent.update_geometry(offset = 20)

	#CheckButton class definition
	class CheckButton:

		def __init__(self, parent, label, on):

			self.var = IntVar()
			self.interactions = []
			self.check = Checkbutton(parent.root, variable = self.var, text = label)
			self.parent = parent

			if on:

				self.var.set(1)

			self.check.pack()
			parent.update_geometry(offset = 25)

		#Add interaction between Sliders and Checkboxes; updates the GUI class dict
		def add_interaction(self, interactions):
			

			if type(interactions) is not GUI.Slider:

				rospy.logerr("This interaction is not supported")
				return

			self.interactions.append(interactions)

			self.parent.interactions.update({self:self.interactions})







	

