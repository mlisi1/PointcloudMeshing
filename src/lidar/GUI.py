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
		self.dict = {'int':IntVar, 'string':StringVar, 'float':DoubleVar}
		self.interactions = {}
		self.pipeline = None
		self.interfaces = []

	#Element creation functions
	def create_slider(self, name, var_type, from_, to, increment, def_val = None):

		return GUI.Slider(self, name, var_type, from_, to, increment, def_val)

	def add_play_button(self):		

		return GUI.PlayButton(self)

	def add_spacer(self, empty = False):

		return GUI.Spacer(self, empty)

	def add_checkbutton(self, label, on = True):

		return GUI.CheckButton(self, label, on)

	def add_button(self, text):

		return GUI.Button(self, text)

	def add_spinbox(self, var, from_, to, increment, def_val = None):

		return GUI.Spinbox(self, var, from_, to, increment, def_val)


	def PipelineICP(self):

		self.pipeline = GUI.Pipeline_ICP(self)
		return self.pipeline


	#Function that updates the geometry of the window; it gets called in every subclass __init__()
	def update_geometry(self, offset):
	

		if self.pipeline is None:
			self.height += offset
			self.root.geometry('160x'+str(self.height))

		else:

			self.height += offset
			total = self.height + self.pipeline.height
			self.root.geometry(str(self.pipeline.width)+'x'+str(total))

	#Simple function to enable/disable sliders given the checkboxes values
	def handle_interactions(self):

		if len(self.interfaces) >0:

			for el in self.interfaces:

				el.update()

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

		if __name__ == '__main__':


			self.handle_interactions()
			self.root.update_idletasks()
			self.root.update()

		else:

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
			self.scale = Scale(parent.root, 
				variable = self.var, 
				resolution = increment, 
				from_ = from_, 
				to = to,
				orient = HORIZONTAL,
				label = name)
			
			#Set default value on init
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
			self.GUI_element = Checkbutton(parent.root, variable = self.var, text = label)
			self.parent = parent

			#Set default value on init
			if on:

				self.var.set(1)

			else:

				self.var.set(0)

			self.GUI_element.pack()
			parent.update_geometry(offset = 25)

		#Add interaction between Sliders and Checkboxes; updates the GUI class dict
		def add_interaction(self, interactions):
			

			if type(interactions) is not GUI.Slider:

				rospy.logerr("This interaction is not supported")
				return

			self.interactions.append(interactions)

			self.parent.interactions.update({self:self.interactions})

		#Check value method
		def check(self):

			return (self.var.get()==1)

			
	#Generic Button class definition
	class Button:

		def __init__(self, parent, text):

			self.button = Button(parent.root, text = text)

			self.button.pack()
			parent.update_geometry(offset = 25)

		#Assign function to the button
		def add_command(self, function):

			self.button.config(command = function)


	#Spinbox class definition
	class Spinbox:

		def __init__(self, parent, var, from_, to, increment, def_val):

			self.var = parent.dict[var]()
			self.spinbox = Spinbox(parent.root, 
				from_ = from_, 
				to = to, 
				increment = increment, 
				textvariable = self.var)

			#Assign defualt value
			if def_val is not None:

				self.var.set(def_val)

			self.spinbox.pack()
			parent.update_geometry(offset = 30)



	#WIP
	#PipelineICP class; not yet fully implemented
	class Pipeline_ICP:

		def __init__(self, parent):


			self.parent = parent
			# parent.root.geometry('200x'+str(parent.height+50))
			self.pipelines = ['Colored ICP', 'Point to Plane', 'Point to Point']
			self.height = 50
			self.width = 200
			self.choice = StringVar()
			self.choice.set(self.pipelines[0])
			self.option = OptionMenu(parent.root, self.choice, *self.pipelines).place(x = 5, y = parent.height + 10)
			self.add_button = Button(parent.root, text = 'Add').place(x = 140, y = parent.height + 11)
			parent.update_geometry(0)





	
if __name__ == '__main__':
   

	GUI = GUI('Debug')
	GUI.add_button('text')
	GUI.add_button('text')
	GUI.add_button('text')
	# GUI.add_spacer(True)

	pipeline = GUI.PipelineICP()

	GUI.add_button('text')



	# filt = GUI.FilterInterface('Filter', True)

	# filt.create_slider('Decimate', 'int', 2,8,1, def_val = 6)
	# filt.create_slider('Decimate 2', 'int', 2,8,1, def_val = 6)




	while True:


		GUI.tk_routine()