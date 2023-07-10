// This file is part of V-REP, the Virtual Robot Experimentation Platform.
// 
// Copyright 2006-2015 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// V-REP is dual-licensed, under the terms of EITHER (at your option):
//   1. V-REP commercial license (contact us for details)
//   2. GNU GPL (see below)
// 
// GNU GPL license:
// -------------------------------------------------------------------
// V-REP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// V-REP IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// 
// See the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with V-REP.  If not, see <http://www.gnu.org/licenses/>.
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.2.2 Rev1 on September 5th 2015

#if !defined(V_REPCONST_INCLUDED_)
#define V_REPCONST_INCLUDED_

#define VREP_PROGRAM_VERSION_NB 30202
#define VREP_PROGRAM_VERSION "3.2.2."

#define VREP_PROGRAM_REVISION_NB 1
#define VREP_PROGRAM_REVISION "(rev. 1)"

/* Scene object types. Values are serialized */
enum { 
		sim_object_shape_type=0,
		sim_object_joint_type,
		sim_object_graph_type,
		sim_object_camera_type,
		sim_object_dummy_type,
		sim_object_proximitysensor_type,
		sim_object_reserved1,
		sim_object_reserved2,
		sim_object_path_type,
		sim_object_visionsensor_type,
		sim_object_volume_type,
		sim_object_mill_type,
		sim_object_forcesensor_type,
		sim_object_light_type,
		sim_object_mirror_type,
		sim_object_type_end=sim_object_path_type+100
};

/* General object types. Values are serialized */
enum {
	sim_appobj_object_type=sim_object_type_end+1,
	sim_appobj_collision_type,
	sim_appobj_distance_type,
	sim_appobj_simulation_type,
	sim_appobj_ik_type,
	sim_appobj_constraintsolver_type,
	sim_appobj_collection_type,
	sim_appobj_ui_type,
	sim_appobj_script_type,
	sim_appobj_pathplanning_type,
	sim_appobj_RESERVED_type,
	sim_appobj_texture_type,
	sim_appobj_motionplanning_type
};

/* Ik calculation methods. Values are serialized */
enum {
		sim_ik_pseudo_inverse_method=0,
		sim_ik_damped_least_squares_method,
		sim_ik_jacobian_transpose_method
};

/* Ik constraints. Values are serialized */
enum {
		sim_ik_x_constraint=1,
		sim_ik_y_constraint=2,
		sim_ik_z_constraint=4,
		sim_ik_alpha_beta_constraint=8,
		sim_ik_gamma_constraint=16,
		sim_ik_avoidance_constraint=64
};

/* Ik calculation results */
enum {
	sim_ikresult_not_performed=0,
	sim_ikresult_success,
	sim_ikresult_fail
};

/* Scene object sub-types. Values are serialized */
enum {
		/* Light sub-types: */
		sim_light_omnidirectional_subtype=1,
		sim_light_spot_subtype,
		sim_light_directional_subtype,
		/* Joint sub-types: */
		sim_joint_revolute_subtype=10,
		sim_joint_prismatic_subtype,
		sim_joint_spherical_subtype,
		/* Shape sub-types: */
		sim_shape_simpleshape_subtype=20,
		sim_shape_multishape_subtype,
		/* Proximity sensor sub-types: */
		sim_proximitysensor_pyramid_subtype=30,
		sim_proximitysensor_cylinder_subtype,
		sim_proximitysensor_disc_subtype,
		sim_proximitysensor_cone_subtype,
		sim_proximitysensor_ray_subtype,
		/* Mill sub-types: */
		sim_mill_pyramid_subtype=40,
		sim_mill_cylinder_subtype,
		sim_mill_disc_subtype,
		sim_mill_cone_subtype,
		/* No sub-type: */
		sim_object_no_subtype=200
};

enum { /* Scene object main properties (serialized): */
		sim_objectspecialproperty_collidable					=0x0001,
		sim_objectspecialproperty_measurable					=0x0002,
		/*reserved						=0x0004, */
		/*reserved						=0x0008, */
		sim_objectspecialproperty_detectable_ultrasonic			=0x0010,
		sim_objectspecialproperty_detectable_infrared			=0x0020,
		sim_objectspecialproperty_detectable_laser				=0x0040,
		sim_objectspecialproperty_detectable_inductive			=0x0080,
		sim_objectspecialproperty_detectable_capacitive			=0x0100,
		sim_objectspecialproperty_renderable					=0x0200,
		sim_objectspecialproperty_detectable_all				=sim_objectspecialproperty_detectable_ultrasonic|sim_objectspecialproperty_detectable_infrared|sim_objectspecialproperty_detectable_laser|sim_objectspecialproperty_detectable_inductive|sim_objectspecialproperty_detectable_capacitive,
		sim_objectspecialproperty_cuttable						=0x0400,
		sim_objectspecialproperty_pathplanning_ignored			=0x0800
};

enum { /* Model properties (serialized): */
		sim_modelproperty_not_collidable				=0x0001,
		sim_modelproperty_not_measurable				=0x0002,
		sim_modelproperty_not_renderable				=0x0004,
		sim_modelproperty_not_detectable				=0x0008,
		sim_modelproperty_not_cuttable					=0x0010,
		sim_modelproperty_not_dynamic					=0x0020,
		sim_modelproperty_not_respondable				=0x0040, /* cannot be selected if sim_modelproperty_not_dynamic is not selected */
		sim_modelproperty_not_reset						=0x0080, /* Model is not reset at simulation end. This flag is cleared at simulation end */
		sim_modelproperty_not_visible					=0x0100, /* Whole model is invisible, independent of local visibility settings */
		sim_modelproperty_scripts_inactive				=0x0200, /* All scripts in the model will not be executed */
		sim_modelproperty_not_model						=0xf000  /* object is not a model */
};

enum { /* Check the documentation instead of comments below!! */
		/* Following messages are dispatched to the Lua-message container: */
		sim_message_ui_button_state_change=0,	/* a UI button, slider, etc. changed (due to a user's action). aux[0]=UI handle, aux[1]=button handle, aux[2]=button attributes, aux[3]=slider position (if slider) */
		sim_message_reserved9,					/* Do not use */
		sim_message_object_selection_changed,
		sim_message_reserved10,					/* do not use */
		sim_message_model_loaded,
		sim_message_reserved11,					/* do not use */
		sim_message_keypress,					/* a key was pressed while the focus was on a page (aux[0]=key, aux[1]=ctrl and shift key state) */
		sim_message_bannerclicked,				/* a banner was clicked (aux[0]=banner ID) */
		sim_message_scene_loaded,				/* a scene was loaded */

		sim_message_prox_sensor_select_down,	/* a "geometric" click select (mouse down) was registered. Enable with sim_intparam_prox_sensor_select_down. aux[0]=objectID, aux2[0-2]=pt coord, aux2[3-5]=pt normal vector */
		sim_message_prox_sensor_select_up,		/* a "geometric" click select (mouse up) was registered. Enable with sim_intparam_prox_sensor_select_up. aux[0]=objectID, aux2[0-2]=pt coord, aux2[3-5]=pt normal vector */
		sim_message_pick_select_down,			/* a "pick" click select (mouse down) was registered. aux[0]=objectID */


		/* Following messages are dispatched only to the C-API (not available from Lua): */
		sim_message_for_c_api_only_start=0x0100,			/* Do not use */
		sim_message_reserved1,								/* Do not use */
		sim_message_reserved2,								/* Do not use */
		sim_message_reserved3,								/* Do not use */


		sim_message_eventcallback_scenesave,				/* about to save a scene */
		sim_message_eventcallback_modelsave,				/* about to save a model (current selection will be saved) */
		sim_message_eventcallback_moduleopen,				/* called when simOpenModule in Lua is called */
		sim_message_eventcallback_modulehandle,				/* called when simHandleModule in Lua is called with argument false */
		sim_message_eventcallback_moduleclose,				/* called when simCloseModule in Lua is called */
		sim_message_reserved4,								/* Do not use */
		sim_message_reserved5,								/* Do not use */
		sim_message_reserved6,								/* Do not use */
		sim_message_reserved7,								/* Do not use */
		sim_message_eventcallback_instancepass,				/* Called once every main application loop pass. auxiliaryData[0] contains event flags of events that happened since last time: */
															/* bit0 set: object(s) erased */
															/* bit1 set: object(s) created */
															/* bit2 set: model loaded */
															/* bit3 set: scene loaded */
															/* bit4 set: undo called */
															/* bit5 set: redo called */
															/* bit6 set: scene switched (similar to scene loaded, basically: scene content completely changed) */
															/* bit7 set: edit mode active. This is not an event flag, but a state flag */
															/* bit8 set: object(s) scaled */
															/* bit9 set: selection state changed. (different objects are selected now) */
															/* bit10 set: key pressed */
															/* bit11 set: simulation started */
															/* bit12 set: simulation ended */

		sim_message_eventcallback_broadcast,
		sim_message_eventcallback_imagefilter_enumreset,
		sim_message_eventcallback_imagefilter_enumerate,
		sim_message_eventcallback_imagefilter_adjustparams,
		sim_message_eventcallback_imagefilter_reserved,
		sim_message_eventcallback_imagefilter_process,

		sim_message_eventcallback_reserved1, /* do not use */
		sim_message_eventcallback_reserved2, /* do not use */
		sim_message_eventcallback_reserved3, /* do not use */
		sim_message_eventcallback_reserved4, /* do not use */

		sim_message_eventcallback_abouttoundo,		/* the undo button was hit and a previous state is about to be restored */
		sim_message_eventcallback_undoperformed,	/* the undo button was hit and a previous state restored */
		sim_message_eventcallback_abouttoredo,		/* the redo button was hit and a future state is about to be restored  */
		sim_message_eventcallback_redoperformed,	/* the redo button was hit and a future state restored  */
		sim_message_eventcallback_scripticondblclick, /* scipt icon was double clicked.  (aux[0]=object handle associated with script, set replyData[0] to 1 if script should not be opened)  */
		sim_message_eventcallback_simulationabouttostart,
		sim_message_eventcallback_simulationended,

		sim_message_eventcallback_reserved5,			/* do not use */
		sim_message_eventcallback_keypress,			/* a key was pressed while the focus was on a page (aux[0]=key, aux[1]=ctrl and shift key state) */
		sim_message_eventcallback_modulehandleinsensingpart, /* called when simHandleModule in Lua is called with argument true */

		sim_message_eventcallback_renderingpass, /* called just before the scene is rendered. From the UI thread! */
		sim_message_eventcallback_bannerclicked, /* called when a banner was clicked (aux[0]=banner ID) */
		sim_message_eventcallback_menuitemselected, /* auxiliaryData[0] indicates the handle of the item, auxiliaryData[1] indicates the state of the item */
		sim_message_eventcallback_refreshdialogs, /* aux[0]=refresh degree (0=light, 1=medium, 2=full) */
		sim_message_eventcallback_sceneloaded,
		sim_message_eventcallback_modelloaded,
		sim_message_eventcallback_instanceswitch,


		sim_message_eventcallback_guipass,
		sim_message_eventcallback_mainscriptabouttobecalled,

		sim_message_eventcallback_rmlposition, /* the command simRMLPosition was called. The appropriate plugin should handle the call */
		sim_message_eventcallback_rmlvelocity, /* the command simRMLVelocity was called. The appropriate plugin should handle the call */

		sim_message_eventcallback_meshcalculationplugin, /* to interact with the mesh calculation plugin */
		sim_message_eventcallback_dynamicsplugin, /* deprecated */
		sim_message_eventcallback_pathplanningplugin, /* to interact with the path planning plugin */
		sim_message_eventcallback_colladaplugin, /* to interact with the collada plugin */

		sim_message_eventcallback_opengl, /* a simple callback at different rendering stages */
		sim_message_eventcallback_openglframe, /* a callback with the full rendered opengl frame data (that can be modified then returned) */
		sim_message_eventcallback_openglcameraview, /* a callback with the rendered opengl view data (that can be modified then returned) */

		sim_message_eventcallback_proxsensorselectdown, /* a "geometric" click select (mouse down) was registered. Enable with sim_intparam_prox_sensor_select_down. aux[0]=objectID, customData[0-2]=pt coord (floats), customData[3-5]=pt normal vector (floats)*/
		sim_message_eventcallback_proxsensorselectup, /* a "geometric" click select (mouse up) was registered. Enable with sim_intparam_prox_sensor_select_down. aux[0]=objectID, customData[0-2]=pt coord (floats), customData[3-5]=pt normal vector (floats)*/
		sim_message_eventcallback_pickselectdown, /* a "pick" click select (mouse down) was registered. aux[0]=objectID */

		sim_message_eventcallback_rmlpos, /* the command simRMLPos was called. The appropriate plugin should handle the call */
		sim_message_eventcallback_rmlvel, /* the command simRMLVel was called. The appropriate plugin should handle the call */
		sim_message_eventcallback_rmlstep, /* the command simRMLStep was called. The appropriate plugin should handle the call */
		sim_message_eventcallback_rmlremove, /* the command simRMLRemove was called. The appropriate plugin should handle the call */
		sim_message_eventcallback_rmlinfo, /* used internally */

		sim_message_eventcallback_beforerendering, /* called just before the scene is rendered. From the main SIM thread! */

		sim_message_eventcallback_extrenderer_start,
		sim_message_eventcallback_extrenderer_light,
		sim_message_eventcallback_extrenderer_mesh,
		sim_message_eventcallback_extrenderer_stop,

		sim_message_eventcallback_dynamicspluginbullet, /* to interact with the dynamics calculation plugin, Bullet */
		sim_message_eventcallback_dynamicspluginode, /* to interact with the dynamics calculation plugin, ODE */
		sim_message_eventcallback_dynamicspluginvortex, /* to interact with the dynamics calculation plugin, Vortex */
		sim_message_eventcallback_dynamicspluginnewton, /* to interact with the dynamics calculation plugin, Newton */
		sim_message_eventcallback_dynamicspluginreserved2, /* for future extensions */
		sim_message_eventcallback_dynamicspluginreserved3, /* for future extensions */
		sim_message_eventcallback_dynamicspluginreserved4, /* for future extensions */
		sim_message_eventcallback_dynamicspluginreserved5, /* for future extensions */
		sim_message_eventcallback_dynamicspluginreserved6, /* for future extensions */
		sim_message_eventcallback_dynamicspluginreserved7, /* for future extensions */
		sim_message_eventcallback_dynamicspluginreserved8, /* for future extensions */
		sim_message_eventcallback_dynamicspluginreserved9, /* for future extensions */

		sim_message_eventcallback_extrenderer_triangles,


		sim_message_simulation_start_resume_request=0x1000,
		sim_message_simulation_pause_request,
		sim_message_simulation_stop_request

};

// Rendering attributes:
enum {
		sim_displayattribute_renderpass		=0x0001,
		sim_displayattribute_depthpass		=0x0002,
		sim_displayattribute_pickpass		=0x0004,
		sim_displayattribute_selected		=0x0008,
		sim_displayattribute_groupselection	=0x0010,
		sim_displayattribute_mainselection	=0x0020,
		sim_displayattribute_forcewireframe	=0x0040,
		sim_displayattribute_forbidwireframe=0x0080,
		sim_displayattribute_forbidedges	=0x0100,
		sim_displayattribute_originalcolors	=0x0200,
		sim_displayattribute_ignorelayer	=0x0400,
		sim_displayattribute_forvisionsensor	=0x0800,
		sim_displayattribute_colorcodedpickpass	=0x1000,
		sim_displayattribute_colorcoded			=0x2000,
		sim_displayattribute_trianglewireframe	=0x4000, 
		sim_displayattribute_simplifyasboundingbox	=0x8000,
		sim_displayattribute_thickEdges				=0x10000,
		sim_displayattribute_dynamiccontentonly		=0x20000,
		sim_displayattribute_mirror					=0x40000,
		sim_displayattribute_useauxcomponent		=0x80000,
		sim_displayattribute_ignorerenderableflag	=0x100000,
		sim_displayattribute_noopenglcallbacks		=0x200000,
		sim_displayattribute_FREE					=0x400000,
		sim_displayattribute_noghosts				=0x800000,
		sim_displayattribute_nopointclouds			=0x1000000,
		sim_displayattribute_nodrawingobjects		=0x2000000,
		sim_displayattribute_noparticles			=0x4000000,
		sim_displayattribute_colorcodedtriangles	=0x8000000
};

enum { /* Scene object properties. Combine with the | operator */
	sim_objectproperty_reserved1	=0x0000,
	sim_objectproperty_reserved2	=0x0001,
	sim_objectproperty_reserved3	=0x0002,
	sim_objectproperty_reserved4	=0x0003,
	sim_objectproperty_reserved5	=0x0004, /* formely sim_objectproperty_visible */
	sim_objectproperty_reserved6	=0x0008, /* formely sim_objectproperty_wireframe */
	sim_objectproperty_collapsed				=0x0010,
	sim_objectproperty_selectable				=0x0020,
	sim_objectproperty_reserved7				=0x0040,
	sim_objectproperty_selectmodelbaseinstead	=0x0080,
	sim_objectproperty_dontshowasinsidemodel	=0x0100,
	/* reserved									=0x0200, */
	sim_objectproperty_canupdatedna				=0x0400,
	sim_objectproperty_selectinvisible			=0x0800,
	sim_objectproperty_depthinvisible			=0x1000
};

enum { /* type of arguments (input and output) for custom lua commands */
	sim_lua_arg_nil=0,			
	sim_lua_arg_bool,		
	sim_lua_arg_int,
	sim_lua_arg_float,
	sim_lua_arg_string,
	sim_lua_arg_invalid,
	sim_lua_arg_charbuff,
	sim_lua_arg_double,
	sim_lua_arg_table=8
	/* SIM_LUA_ARG_NIL_ALLOWED=65536 is defined and used in CLuaFunctionData.h. This flag is reserved here. */
};

enum { /* custom user interface properties. Values are serialized. */
	sim_ui_property_visible						=0x0001,
	sim_ui_property_visibleduringsimulationonly	=0x0002,
	sim_ui_property_moveable					=0x0004,
	sim_ui_property_relativetoleftborder		=0x0008,
	sim_ui_property_relativetotopborder			=0x0010,
	sim_ui_property_fixedwidthfont				=0x0020,
	sim_ui_property_systemblock					=0x0040,
	sim_ui_property_settocenter					=0x0080,
	sim_ui_property_rolledup					=0x0100,
	sim_ui_property_selectassociatedobject		=0x0200,
	sim_ui_property_visiblewhenobjectselected	=0x0400,
	sim_ui_property_systemblockcanmovetofront	=0x0800,
	sim_ui_property_pauseactive					=0x1000
};

enum { /* button properties. Values are serialized. */
	sim_buttonproperty_button				=0x0000,
	sim_buttonproperty_label				=0x0001,
	sim_buttonproperty_slider				=0x0002,
	sim_buttonproperty_editbox				=0x0003,
	sim_buttonproperty_staydown				=0x0008,
	sim_buttonproperty_enabled				=0x0010,
	sim_buttonproperty_borderless			=0x0020,
	sim_buttonproperty_horizontallycentered	=0x0040,
	sim_buttonproperty_ignoremouse			=0x0080,
	sim_buttonproperty_isdown				=0x0100,
	sim_buttonproperty_transparent			=0x0200,
	sim_buttonproperty_nobackgroundcolor	=0x0400,
	sim_buttonproperty_rollupaction			=0x0800,
	sim_buttonproperty_closeaction			=0x1000,
	sim_buttonproperty_verticallycentered	=0x2000,
	sim_buttonproperty_downupevent			=0x4000
};

enum { /* Simulation status */
	sim_simulation_stopped						=0x00,								/* Simulation is stopped */
	sim_simulation_paused						=0x08,								/* Simulation is paused */
	sim_simulation_advancing					=0x10,								/* Simulation is advancing */
	sim_simulation_advancing_firstafterstop		=sim_simulation_advancing|0x00,		/* First simulation pass (1x) */
	sim_simulation_advancing_running			=sim_simulation_advancing|0x01,		/* Normal simulation pass (>=1x) */
	/* reserved									=sim_simulation_advancing|0x02, */
	sim_simulation_advancing_lastbeforepause	=sim_simulation_advancing|0x03,		/* Last simulation pass before pause (1x) */
	sim_simulation_advancing_firstafterpause	=sim_simulation_advancing|0x04,		/* First simulation pass after pause (1x) */
	sim_simulation_advancing_abouttostop		=sim_simulation_advancing|0x05,		/* "Trying to stop" simulation pass (>=1x) */
	sim_simulation_advancing_lastbeforestop		=sim_simulation_advancing|0x06		/* Last simulation pass (1x) */
};

enum { /* Script execution result (first return value) */
	sim_script_no_error					=0,
	sim_script_main_script_nonexistent	=1,
	sim_script_main_script_not_called	=2,
	sim_script_reentrance_error			=4,
	sim_script_lua_error				=8,
	sim_script_call_error				=16
};

enum { /* Script types (serialized!) */
	sim_scripttype_mainscript=0,
	sim_scripttype_childscript,
	sim_scripttype_addonscript,
	sim_scripttype_addonfunction,
	sim_scripttype_jointctrlcallback,
	sim_scripttype_contactcallback,
	sim_scripttype_customizationscript,
	sim_scripttype_generalcallback,
	sim_scripttype_threaded=0x00f0			/* Combine with sim_scripttype_childscript if you want */
};

enum { /* Main script call types */
	sim_mainscriptcall_initialization=0, /* first time called. Do some initializations */
	sim_mainscriptcall_cleanup, /* last time called. Do some clean-up */
	sim_mainscriptcall_regular /* otherwise */
};

enum { /* Child script call types */
	sim_childscriptcall_initialization=0, /* first time called. Do some initializations */
	sim_childscriptcall_cleanup, /* last time called. Do some clean-up */
	sim_childscriptcall_actuation, /* called in the "actuation" phase. */
	sim_childscriptcall_sensing, /* called in the "sensing" phase. */
	sim_childscriptcall_threaded /* called only by threaded scripts. */
};

enum { /* Customization script call types */
	sim_customizationscriptcall_initialization=0, /* first time called. Do some initializations */
	sim_customizationscriptcall_cleanup, /* last time called. Do some clean-up */
	sim_customizationscriptcall_nonsimulation, /* called while simulation not running */
	sim_customizationscriptcall_lastbeforesimulation, /* called just before simulation starts */
	sim_customizationscriptcall_firstaftersimulation, /* called just after simulation ended */
	sim_customizationscriptcall_simulationactuation, /* called in the "actuation" phase of the main script. Called only if the script is flagged as sim_customizationscriptattribute_activeduringsimulation */
	sim_customizationscriptcall_simulationsensing, /* called in the "sensing" phase of the main script. Called only if the script is flagged as sim_customizationscriptattribute_activeduringsimulation */
	sim_customizationscriptcall_simulationpause, /* called while simulation is paused */
	sim_customizationscriptcall_simulationpausefirst, /* called just after a simulation was paused */
	sim_customizationscriptcall_simulationpauselast /* called just before a simulation is unpaused */
};

enum { /* Script attributes */
	sim_customizationscriptattribute_activeduringsimulation=0, /* boolean attribute */
	sim_scriptattribute_executionorder, /* integer attribute, see sim_scriptexecorder_normal, etc. */
	sim_scriptattribute_executioncount, /* integer attribute */
	sim_childscriptattribute_automaticcascadingcalls, /* boolean attribute */
	sim_childscriptattribute_enabled /* boolean attribute */
};

enum { /* Script execution order, is serialized */
	sim_scriptexecorder_first=0,
	sim_scriptexecorder_normal,
	sim_scriptexecorder_last
};

enum { /* Threaded script resume location */
	sim_scriptthreadresume_allnotyetresumed	=-1,
	sim_scriptthreadresume_default			=0,
	sim_scriptthreadresume_actuation_first	=1,
	sim_scriptthreadresume_actuation_last	=2,
	sim_scriptthreadresume_sensing_first	=3,
	sim_scriptthreadresume_sensing_last		=4,
	sim_scriptthreadresume_custom			=5
};

enum { /* General callback IDs */
	sim_callbackid_rossubscriber			=1,
	sim_callbackid_dynstep					=2,
	sim_callbackid_userdefined				=1000
};


enum { /* API call error messages */
	sim_api_error_report =1,
	sim_api_error_output =2,
	sim_api_warning_output =4,

	/* for backward compatibility */
	sim_api_errormessage_ignore	=0,	/* does not memorize nor output errors */
	sim_api_errormessage_report	=1,	/* memorizes errors (default for C-API calls) */
	sim_api_errormessage_output	=2  /* memorizes and outputs errors (default for Lua-API calls) */
};

enum { /* special argument of some functions: */
	sim_handle_all						=-2,
	sim_handle_all_except_explicit		=-3,
	sim_handle_self						=-4,
	sim_handle_main_script				=-5,
	sim_handle_tree						=-6,
	sim_handle_chain					=-7,
	sim_handle_single					=-8,
	sim_handle_default					=-9,
	sim_handle_all_except_self			=-10,
	sim_handle_parent					=-11,
	sim_handle_scene					=-12
};

enum { /* special handle flags: */
	sim_handleflag_assembly				=0x00400000,
	sim_handleflag_togglevisibility		=0x00400000,
	sim_handleflag_model				=0x00800000,
	sim_handleflag_rawvalue				=0x01000000
};

enum { /* distance calculation methods: (serialized) */
	sim_distcalcmethod_dl=0,
	sim_distcalcmethod_dac,
	sim_distcalcmethod_max_dl_dac,
	sim_distcalcmethod_dl_and_dac,
	sim_distcalcmethod_sqrt_dl2_and_dac2,
	sim_distcalcmethod_dl_if_nonzero,
	sim_distcalcmethod_dac_if_nonzero
};

enum { /* Generic dialog styles: */
	sim_dlgstyle_message		=0,
	sim_dlgstyle_input,
	sim_dlgstyle_ok,
	sim_dlgstyle_ok_cancel,
	sim_dlgstyle_yes_no,
	sim_dlgstyle_dont_center	=32 /* can be combined with one of above values. Only with this flag can the position of the related UI be set just after dialog creation  */
};

enum { /* Generic dialog return values: */
	sim_dlgret_still_open=0,
	sim_dlgret_ok,
	sim_dlgret_cancel,
	sim_dlgret_yes,
	sim_dlgret_no
};

enum { /* Path properties: */
	sim_pathproperty_show_line				=0x0001,
	sim_pathproperty_show_orientation		=0x0002,
	sim_pathproperty_closed_path			=0x0004,
	sim_pathproperty_automatic_orientation	=0x0008,
	sim_pathproperty_invert_velocity_deprecated		=0x0010,
	sim_pathproperty_infinite_acceleration_deprecated	=0x0020,
	sim_pathproperty_flat_path				=0x0040,
	sim_pathproperty_show_position			=0x0080,
	sim_pathproperty_auto_velocity_profile_translation_deprecated	=0x0100,
	sim_pathproperty_auto_velocity_profile_rotation_deprecated		=0x0200,
	sim_pathproperty_endpoints_at_zero_deprecated					=0x0400,
	sim_pathproperty_keep_x_up							=0x0800
};

enum { /* drawing objects: */
	/* following are mutually exclusive: */
	sim_drawing_points=0,			/* 3 values per point (point size in pixels) */
	sim_drawing_lines,				/* 6 values per line (line size in pixels) */
	sim_drawing_triangles,			/* 9 values per triangle */
	sim_drawing_trianglepoints,		/* 6 values per point (3 for triangle position, 3 for triangle normal vector) (triangle size in meters) */
	sim_drawing_quadpoints,			/* 6 values per point (3 for quad position, 3 for quad normal vector) (quad size in meters) */
	sim_drawing_discpoints,			/* 6 values per point (3 for disc position, 3 for disc normal vector) (disc size in meters) */
	sim_drawing_cubepoints,			/* 6 values per point (3 for cube position, 3 for cube normal vector) (cube size in meters) */
	sim_drawing_spherepoints,			/* 3 values per point (sphere size in meters) */

	/* following can be or-combined: */
	sim_drawing_itemcolors				=0x000020, /* +3 values per item (each item has its own ambient color (r,g,b values)). Mutually exclusive with sim_drawing_vertexcolors */
	sim_drawing_vertexcolors			=0x000040, /* +3 values per vertex (each vertex has its own ambient color (r,g,b values). Only for sim_drawing_lines (+6) and for sim_drawing_triangles(+9)). Mutually exclusive with sim_drawing_itemcolors */
	sim_drawing_itemsizes				=0x000080, /* +1 value per item (each item has its own size). Not for sim_drawing_triangles */
	sim_drawing_backfaceculling			=0x000100, /* back faces are not displayed for all items */
	sim_drawing_wireframe				=0x000200, /* all items displayed in wireframe */
	sim_drawing_painttag				=0x000400, /* all items are tagged as paint (for additinal processing at a later stage) */
	sim_drawing_followparentvisibility	=0x000800, /* if the object is associated with a scene object, then it follows that visibility, otherwise it is always visible */
	sim_drawing_cyclic					=0x001000, /* if the max item count was reached, then the first items are overwritten. */
	sim_drawing_50percenttransparency	=0x002000, /* the drawing object will be 50% transparent */
	sim_drawing_25percenttransparency	=0x004000, /* the drawing object will be 25% transparent */
	sim_drawing_12percenttransparency	=0x008000, /* the drawing object will be 12.5% transparent */
	sim_drawing_emissioncolor			=0x010000, /* When used in combination with sim_drawing_itemcolors or sim_drawing_vertexcolors, then the specified colors will be for the emissive component */
	sim_drawing_facingcamera			=0x020000, /* Only for trianglepoints, quadpoints, discpoints and cubepoints. If specified, the normal verctor is calculated to face the camera (each item data requires 3 values less) */
	sim_drawing_overlay					=0x040000, /* When specified, objects are always drawn on top of "regular objects" */
	sim_drawing_itemtransparency		=0x080000,  /* +1 value per item (each item has its own transparency value (0-1)). Not compatible with sim_drawing_vertexcolors */
	sim_drawing_persistent				=0x100000,  /* object is not automatically removed when created via a simulation script */
	sim_drawing_auxchannelcolor1		=0x200000,  /* when specifying the emission color component in the simAddDrawingObject function, 6 values should be specified instead of 3: 3 for the emission component, and 3 for the aux channel component */
	sim_drawing_auxchannelcolor2		=0x400000,  /* When used in combination with sim_drawing_itemcolors or sim_drawing_vertexcolors, then the specified colors will be for the aux channel component */
};

enum { /* banner values: */
	/* following can be or-combined: */
	sim_banner_left						=0x00001, /* Banners display on the left of the specified point */
	sim_banner_right					=0x00002, /* Banners display on the right of the specified point */
	sim_banner_nobackground				=0x00004, /* Banners have no background rectangle */
	sim_banner_overlay					=0x00008, /* When specified, banners are always drawn on top of "regular objects" */
	sim_banner_followparentvisibility	=0x00010, /* if the object is associated with a scene object, then it follows that visibility, otherwise it is always visible */
	sim_banner_clickselectsparent		=0x00020, /* if the object is associated with a scene object, then clicking the banner will select the scene object */
	sim_banner_clicktriggersevent		=0x00040, /* if the banner is clicked, an event is triggered (sim_message_eventcallback_bannerclicked and sim_message_bannerclicked are generated) */
	sim_banner_facingcamera				=0x00080, /* If specified, the banner will always face the camera by rotating around the banner's vertical axis (y-axis) */
	sim_banner_fullyfacingcamera		=0x00100, /* If specified, the banner will always fully face the camera (the banner's orientation is same as the camera looking at it) */
	sim_banner_backfaceculling			=0x00200, /* If specified, the banner will only be visible from one side */
	sim_banner_keepsamesize				=0x00400, /* If specified, the banner will always appear in the same size. In that case size represents the character height in pixels */
	sim_banner_bitmapfont				=0x00800 /* If specified, a fixed-size bitmap font is used. The text will also always fully face the camera and be right to the specified position. Bitmap fonts are not clickable */
};

enum { /* particle objects: */
	/* following are mutually exclusive: */
	sim_particle_points1=0,			/* 6 values per point (pt1 and pt2. Pt1 is start position, pt2-pt1 is the initial velocity vector). Point is 1 pixel big. Only appearance is a point, internally handled as a perfect sphere */
	sim_particle_points2,			/* 6 values per point. Point is 2 pixel big. Only appearance is a point, internally handled as a perfect sphere */
	sim_particle_points4,			/* 6 values per point. Point is 4 pixel big. Only appearance is a point, internally handled as a perfect sphere */
	sim_particle_roughspheres,		/* 6 values per sphere. Only appearance is rough. Internally a perfect sphere */
	sim_particle_spheres,			/* 6 values per sphere. Internally a perfect sphere */

	/* following can be or-combined: */
	sim_particle_respondable1to4		=0x0020, /* the particles are respondable against shapes (against all objects that have at least one bit 1-4 activated in the global respondable mask) */
	sim_particle_respondable5to8		=0x0040, /* the particles are respondable against shapes (against all objects that have at least one bit 5-8 activated in the global respondable mask) */
	sim_particle_particlerespondable	=0x0080, /* the particles are respondable against each other */
	sim_particle_ignoresgravity			=0x0100, /* the particles ignore the effect of gravity. Not compatible with sim_particle_water */
	sim_particle_invisible				=0x0200, /* the particles are invisible */
	sim_particle_itemsizes				=0x0400, /* +1 value per particle (each particle can have a different size) */
	sim_particle_itemdensities			=0x0800, /* +1 value per particle (each particle can have a different density) */
	sim_particle_itemcolors				=0x1000, /* +3 values per particle (each particle can have a different color) */
	sim_particle_cyclic					=0x2000, /* if the max item count was reached, then the first items are overwritten. */
	sim_particle_emissioncolor			=0x4000, /* When used in combination with sim_particle_itemcolors, then the specified colors will be for the emissive component */
	sim_particle_water					=0x8000, /* the particles are water particles (no weight in the water (i.e. when z<0)). Not compatible with sim_particle_ignoresgravity */
	sim_particle_painttag				=0x10000 /* The particles can be seen by vision sensors (sim_particle_invisible must not be set) */
};

enum { /* custom user interface menu attributes: */
	sim_ui_menu_title		=1,
	sim_ui_menu_minimize	=2,
	sim_ui_menu_close		=4,
	sim_ui_menu_systemblock =8
};

enum { /* Boolean parameters: */
	sim_boolparam_hierarchy_visible=0,
	sim_boolparam_console_visible,
	sim_boolparam_collision_handling_enabled,
	sim_boolparam_distance_handling_enabled,
	sim_boolparam_ik_handling_enabled,
	sim_boolparam_gcs_handling_enabled,
	sim_boolparam_dynamics_handling_enabled,
	sim_boolparam_joint_motion_handling_enabled_deprecated,
	sim_boolparam_path_motion_handling_enabled_deprecated,
	sim_boolparam_proximity_sensor_handling_enabled,
	sim_boolparam_vision_sensor_handling_enabled,
	sim_boolparam_mill_handling_enabled,
	sim_boolparam_browser_visible,
	sim_boolparam_scene_and_model_load_messages,
	sim_reserved0,
	sim_boolparam_shape_textures_are_visible,
	sim_boolparam_display_enabled,
	sim_boolparam_infotext_visible,
	sim_boolparam_statustext_open,
	sim_boolparam_fog_enabled,
	sim_boolparam_rml2_available,
	sim_boolparam_rml4_available,
	sim_boolparam_mirrors_enabled,
	sim_boolparam_aux_clip_planes_enabled,
	sim_boolparam_reserved3,
	sim_boolparam_realtime_simulation,
	sim_boolparam_use_glfinish_cmd,
	sim_boolparam_force_show_wireless_emission,
	sim_boolparam_force_show_wireless_reception,
	sim_boolparam_video_recording_triggered,
	sim_boolparam_reserved1,
	sim_boolparam_reserved2,
	sim_boolparam_threaded_rendering_enabled,
	sim_boolparam_fullscreen,
	sim_boolparam_headless,
	sim_boolparam_hierarchy_toolbarbutton_enabled,
	sim_boolparam_browser_toolbarbutton_enabled,
	sim_boolparam_objectshift_toolbarbutton_enabled,
	sim_boolparam_objectrotate_toolbarbutton_enabled,
	sim_boolparam_force_calcstruct_all_visible,
	sim_boolparam_force_calcstruct_all,
	sim_boolparam_exit_request,
	sim_boolparam_play_toolbarbutton_enabled,
	sim_boolparam_pause_toolbarbutton_enabled,
	sim_boolparam_stop_toolbarbutton_enabled
};

enum { /* Integer parameters: */
	sim_intparam_error_report_mode=0, /* Check sim_api_errormessage_... constants above for valid values */
	sim_intparam_program_version,		/* e.g Version 2.1.4 --> 20104. Can only be read. See also sim_intparam_program_revision */
	sim_intparam_instance_count,	/* do not use anymore (always returns 1 since V-REP 2.5.11) */
	sim_intparam_custom_cmd_start_id, /* can only be read */
	sim_intparam_compilation_version, /* 0=evaluation version, 1=full version, 2=player version. Can only be read */
	sim_intparam_current_page,
	sim_intparam_flymode_camera_handle, /* can only be read */
	sim_intparam_dynamic_step_divider, /* can only be read */
	sim_intparam_dynamic_engine, /* 0=Bullet, 1=ODE. 2=Vortex. 3=Newton. Can only be read */
	sim_intparam_server_port_start, /* can only be read */
	sim_intparam_server_port_range, /* can only be read */
	sim_intparam_visible_layers,
	sim_intparam_infotext_style,
	sim_intparam_settings,
	sim_intparam_edit_mode_type, /* can only be read */
	sim_intparam_server_port_next, /* is initialized at sim_intparam_server_port_start */
	sim_intparam_qt_version, /* version of the used Qt framework */
	sim_intparam_event_flags_read, /* can only be read */
	sim_intparam_event_flags_read_clear, /* can only be read */
	sim_intparam_platform, /* can only be read */
	sim_intparam_scene_unique_id, /* can only be read */
	sim_intparam_work_thread_count, /* 0-256. 0 to disable, -1 to try to automatically set */
	sim_intparam_mouse_x, /* can only be read */
	sim_intparam_mouse_y, /* can only be read */
	sim_intparam_core_count, /* can only be read */
	sim_intparam_work_thread_calc_time_ms,
	sim_intparam_idle_fps,
	sim_intparam_prox_sensor_select_down,
	sim_intparam_prox_sensor_select_up,
	sim_intparam_stop_request_counter,
	sim_intparam_program_revision, /* Can only be read. See also sim_intparam_program_version */
	sim_intparam_mouse_buttons, /* can only be read */
	sim_intparam_dynamic_warning_disabled_mask,
	sim_intparam_simulation_warning_disabled_mask,
	sim_intparam_scene_index, /* can be used to switch to a different instance programmatically */
	sim_intparam_motionplanning_seed,
	sim_intparam_speedmodifier /* can only be used while simulation is not stopped */
};

enum { /* Float parameters: */
	sim_floatparam_rand=0, /* random value (0.0-1.0) */
	sim_floatparam_simulation_time_step,
	sim_floatparam_stereo_distance
};

enum { /* String parameters: */
	sim_stringparam_application_path=0, /* path of V-REP's executable */
	sim_stringparam_video_filename, /* name + path without extension! */
	sim_stringparam_app_arg1, /* specify those args with "vrep -gxxxx" */
	sim_stringparam_app_arg2,
	sim_stringparam_app_arg3,
	sim_stringparam_app_arg4,
	sim_stringparam_app_arg5,
	sim_stringparam_app_arg6,
	sim_stringparam_app_arg7,
	sim_stringparam_app_arg8,
	sim_stringparam_app_arg9,
	sim_stringparam_additional_addonscript_firstscene, /* do not use. Can only be written. */
	sim_stringparam_additional_addonscript, /* do not use. Can only be written. */
	sim_stringparam_scene_path_and_name, /* can only be read */
	sim_stringparam_remoteapi_temp_file_dir /* can only be read */
};

enum { /* Array parameters: */
	sim_arrayparam_gravity=0,
	sim_arrayparam_fog,
	sim_arrayparam_fog_color,
	sim_arrayparam_background_color1,
	sim_arrayparam_background_color2,
	sim_arrayparam_ambient_light,
	sim_arrayparam_random_euler
};

enum { /* User interface elements: */
	sim_gui_menubar						=0x00001,
	sim_gui_popups						=0x00002,
	sim_gui_toolbar1					=0x00004,
	sim_gui_toolbar2					=0x00008,
	sim_gui_hierarchy					=0x00010,
	sim_gui_infobar						=0x00020,
	sim_gui_statusbar					=0x00040,
	sim_gui_scripteditor				=0x00080,
	sim_gui_scriptsimulationparameters	=0x00100,
	sim_gui_dialogs						=0x00200,
	sim_gui_browser						=0x00400,
	sim_gui_all							=0x0ffff,
	sim_gui_headless					=0x10000
};

enum { /* Joint modes: */
	sim_jointmode_passive=0,
	sim_jointmode_motion_deprecated,
	sim_jointmode_ik,
	sim_jointmode_reserved_previously_ikdependent,
	sim_jointmode_dependent,
	sim_jointmode_force
};

enum { /* Navigation and selection modes with the mouse. Lower byte values are mutually exclusive, upper byte bits can be combined */
	sim_navigation_passive					=0x000000,
	sim_navigation_camerashift				=0x000001,
	sim_navigation_camerarotate				=0x000002,
	sim_navigation_camerazoom				=0x000003,
	sim_navigation_cameratilt				=0x000004,
	sim_navigation_cameraangle				=0x000005,
	sim_navigation_camerafly				=0x000006,
	sim_navigation_objectshift				=0x000007,
	sim_navigation_objectrotate				=0x000008,
	sim_navigation_reserved2				=0x000009,
	sim_navigation_reserved3				=0x00000A,
	sim_navigation_reserved4				=0x00000B,
	sim_navigation_reserved5				=0x00000C,
	sim_navigation_reserved6				=0x00000D,
	/* Bit-combine following values and add them to one of above's values for a valid navigation mode: */
	sim_navigation_createpathpoint			=0x000100,
	sim_navigation_clickselection			=0x000200,
	sim_navigation_ctrlselection			=0x000400,
	sim_navigation_shiftselection			=0x000800,
	sim_navigation_camerazoomwheel			=0x001000,
	sim_navigation_camerarotaterightbutton	=0x002000, /* deprecated */
	sim_navigation_camerarotatemiddlebutton	=0x002000
};


enum { /* Reflexxes Motion Library flags */
	/* enable one of following: */
	simrml_phase_sync_if_possible					=0x0000,	
	simrml_only_time_sync							=0x0001,	
	simrml_only_phase_sync							=0x0002,	
	simrml_no_sync									=0x0003,

	/* enable one of following: */
	simrml_keep_target_vel							=0x0000,	
	simrml_recompute_trajectory						=0x0004,	

	/* enable any: */
	simrml_disable_extremum_motion_states_calc		=0x0008,
	simrml_keep_current_vel_if_fallback_strategy	=0x0010
};

enum { /* file dialog styles */
	sim_filedlg_type_load=0,
	sim_filedlg_type_save
};

enum { /* message box styles */
	sim_msgbox_type_info=0,
	sim_msgbox_type_question,
	sim_msgbox_type_warning,
	sim_msgbox_type_critical
};

enum { /* message box buttons */
	sim_msgbox_buttons_ok=0,
	sim_msgbox_buttons_yesno,
	sim_msgbox_buttons_yesnocancel,
	sim_msgbox_buttons_okcancel
};

enum { /* message box return values */
	sim_msgbox_return_cancel=0,
	sim_msgbox_return_no,
	sim_msgbox_return_yes,
	sim_msgbox_return_ok,
	sim_msgbox_return_error
};

enum { /* physics engine */
	sim_physics_bullet=0,
	sim_physics_ode,
	sim_physics_vortex,
	sim_physics_newton
};

enum { /* pure primitives type */
	sim_pure_primitive_none=0,
	sim_pure_primitive_plane,
	sim_pure_primitive_disc,
	sim_pure_primitive_cuboid,
	sim_pure_primitive_spheroid,
	sim_pure_primitive_cylinder,
	sim_pure_primitive_cone,
	sim_pure_primitive_heightfield
};

enum { /* dummy-dummy link types */
	sim_dummy_linktype_dynamics_loop_closure=0,
	sim_dummy_linktype_dynamics_force_constraint,
	sim_dummy_linktype_gcs_loop_closure,
	sim_dummy_linktype_gcs_tip,
	sim_dummy_linktype_gcs_target,
	sim_dummy_linktype_ik_tip_target,
	sim_dummy_linktype_reserved
};

enum { /* texture map modes (serialized) */
	sim_texturemap_plane=0,
	sim_texturemap_cylinder,
	sim_texturemap_sphere,
	sim_texturemap_cube
};

enum { /* path planning types */
	sim_pathplanning_holonomic=0,
	sim_pathplanning_reserved,
	sim_pathplanning_nonholonomic
};

enum { /* Holonomic path planning types */
	sim_holonomicpathplanning_xy=0,		// 2 DoF
	sim_holonomicpathplanning_xg,
	sim_holonomicpathplanning_xyz,		// 3 DoF
	sim_holonomicpathplanning_xyg,
	sim_holonomicpathplanning_abg,
	sim_holonomicpathplanning_xyzg,		// 4 DoF
	sim_holonomicpathplanning_xabg,
	sim_holonomicpathplanning_xyabg,	// 5 DoF
	sim_holonomicpathplanning_xyzabg	// 6 Dof
};

enum { /* resource lock types */
	sim_lock_ui_wants_to_read=0,
	sim_lock_ui_wants_to_write,
	sim_lock_nonui_wants_to_write
};

enum { /* color components */
	sim_colorcomponent_ambient=0,
	sim_colorcomponent_ambient_diffuse=0,
	sim_colorcomponent_diffuse,
	sim_colorcomponent_specular,
	sim_colorcomponent_emission,
	sim_colorcomponent_transparency,
	sim_colorcomponent_auxiliary
};

enum { /* Pov pattern types */
	sim_pov_pattern_null=0,
	sim_pov_white_marble,
	sim_pov_blood_marble,
	sim_pov_blue_agate,
	sim_pov_sapphire_agate,
	sim_pov_brown_agate,
	sim_pov_pink_granite,
	sim_pov_pink_alabaster,
	sim_pov_cherry_wood,
	sim_pov_pine_wood,
	sim_pov_dark_wood,
	sim_pov_tan_wood,
	sim_pov_white_wood,
	sim_pov_tom_wood,
	sim_pov_dmf_wood1,
	sim_pov_dmf_wood2,
	sim_pov_dmf_wood3,
	sim_pov_dmf_wood4,
	sim_pov_dmf_wood5,
	sim_pov_dmf_wood6,
	sim_pov_dmf_light_oak,
	sim_pov_dmf_dark_oak,
	sim_pov_emb_wood1,
	sim_pov_yellow_pine,
	sim_pov_rose_wood,
	sim_pov_sandal_wood,
	sim_pov_glass,
	sim_pov_glass2,
	sim_pov_glass3,
	sim_pov_green_glass,
	sim_pov_light_glass,
	sim_pov_bold_glass,
	sim_pov_wine_bottle,
	sim_pov_beer_bottle,
	sim_pov_ruby_glass,
	sim_pov_blue_glass,
	sim_pov_yellow_glass,
	sim_pov_orange_glass,
	sim_pov_vicks_bottle_glass,
	sim_pov_chrome_metal,
	sim_pov_brass_metal,
	sim_pov_copper_metal,
	sim_pov_bronze_metal,
	sim_pov_silver_metal,
	sim_pov_gold_metal,
	sim_pov_polished_chrome,
	sim_pov_polished_brass,
	sim_pov_new_brass,
	sim_pov_spun_brass,
	sim_pov_brushed_aluminum,
	sim_pov_silver1,
	sim_pov_silver2,
	sim_pov_silver3,
	sim_pov_brass_valley,
	sim_pov_rust,
	sim_pov_rusty_iron,
	sim_pov_soft_silver,
	sim_pov_new_penny,
	sim_pov_tinny_brass,
	sim_pov_gold_nugget,
	sim_pov_aluminum,
	sim_pov_bright_bronze,
	sim_pov_water,
	sim_pov_cork,
	sim_pov_lightning,
	sim_pov_mirror
};

/******************************************
*******************************************
Remote API constants:
*******************************************
*******************************************/

#define SIMX_VERSION 9  /* max is 255!!! */
/* version to 6 for release 3.1.2 */
/* version to 7 for release 3.1.3 */
/* version to 8 for release AFTER 3.1.3 */

/*
Messages sent/received look like this:
-Message header (SIMX_HEADER_SIZE in size)
-Command header 1 (SIMX_SUBHEADER_SIZE in size)
-Command data 1
-Pure data 1
-Command header 2 (SIMX_SUBHEADER_SIZE in size)
-Command data 2
-Pure data 2
- ...
-Command header n (SIMX_SUBHEADER_SIZE in size)
-Command data n
-Pure data n
*/

/* Remote API message header structure: */
#define SIMX_HEADER_SIZE 18
#define simx_headeroffset_crc 0				/* 1 simxUShort. Generated by the client or server. The CRC for the message */
#define simx_headeroffset_version 2			/* 1 byte. Generated by the client or server. The version of the remote API software */
#define simx_headeroffset_message_id 3		/* 1 simxInt. Generated by the client (and used in a reply by the server) */
#define simx_headeroffset_client_time 7		/* 1 simxInt. Client time stamp generated by the client (and sent back by the server) */
#define simx_headeroffset_server_time 11	/* 1 simxInt. Generated by the server when a reply is generated. The server timestamp */
#define simx_headeroffset_scene_id 15		/* 1 simxUShort. Generated by the server. A unique ID identifying the scene currently displayed */
#define simx_headeroffset_server_state 17	/* 1 byte. Generated by the server. Bit coded: 0 set --> simulation not stopped, 1 set --> simulation paused, 2 set --> real-time switch on, 3-5: edit mode type (0=no edit mode, 1=triangle, 2=vertex, 3=edge, 4=path, 5=UI) */ 

/* Remote API command header: */
#define SIMX_SUBHEADER_SIZE 26
#define simx_cmdheaderoffset_mem_size 0			/* 1 simxInt. Generated by the client or server. The buffer size of the command. */
#define simx_cmdheaderoffset_full_mem_size 4	/* 1 simxInt. Generated by the client or server. The full buffer size of the command (applies to split chunks). */
#define simx_cmdheaderoffset_pdata_offset0 8	/* 1 simxUShort. Generated by the client or server. The amount of data that is part of the command identification. */
#define simx_cmdheaderoffset_pdata_offset1 10	/* 1 simxInt. Generated by the client or server. The amount of shift of the pure data buffer (applies to split chunks). */
#define simx_cmdheaderoffset_cmd 14				/* 1 simxInt. Generated by the client (and used in a reply by the server). The command, combined with the operation mode of the command. */
#define simx_cmdheaderoffset_delay_or_split 18	/* 1 simxUShort. Generated by the client or server. The amount of delay in ms of a streaming command, or the max. pure data size to send at once (applies to split commands). */
#define simx_cmdheaderoffset_sim_time 20		/* 1 simxInt. Generated by the server. The simulation time (in ms) when the command was executed (or 0 if simulation is not running) */
#define simx_cmdheaderoffset_status 24			/* 1 byte. Generated by the server. (1: bit 0 is set --> error in function execution on server side). The client writes bit 1 if command cannot be overwritten*/
#define simx_cmdheaderoffset_reserved 25		/* 1 byte. Not yet used */

/* All command codes (followed by operation mode codes) */
enum {	simx_cmdnull_start				=0,
		/* from here on, commands are only identified by their code */
		simx_cmd_synchronous_enable,
		simx_cmd_synchronous_disable,
		simx_cmd_synchronous_next,
		simx_cmd_get_last_errors,
		simx_cmd_close_scene,
		simx_cmd_get_object_selection,
		simx_cmd_reserved1,
		simx_cmd_reserved2,
		simx_cmd_create_dummy,
		simx_cmd_kill_connection,

		simx_cmdnull_custom_start		=0x000800,

		simx_cmd4bytes_start			=0x001000, 
		/* from here on, commands are also identified by additional 4 bytes */
		simx_cmd_get_joint_position,
		simx_cmd_set_joint_position,
		simx_cmd_get_vision_sensor_image_bw,
		simx_cmd_get_vision_sensor_image_rgb,
		simx_cmd_set_vision_sensor_image_bw,
		simx_cmd_set_vision_sensor_image_rgb,
		simx_cmd_start_pause_stop_simulation,
		simx_cmd_set_joint_target_velocity,
		simx_cmd_read_proximity_sensor,
		simx_cmd_get_joint_matrix,
		simx_cmd_set_spherical_joint_matrix,
		simx_cmd_set_joint_target_position,
		simx_cmd_get_joint_force,
		simx_cmd_set_joint_force,
		simx_cmd_read_force_sensor,
		simx_cmd_break_force_sensor,
		simx_cmd_read_vision_sensor,
		simx_cmd_get_object_parent,
		simx_cmd_spotcanbeused5,
		simx_cmd_aux_console_close,
		simx_cmd_aux_console_print,
		simx_cmd_aux_console_show,
		simx_cmd_get_vision_sensor_depth_buffer,
		simx_cmd_get_object_orientation,
		simx_cmd_get_object_position,
		simx_cmd_set_object_orientation,
		simx_cmd_set_object_position,
		simx_cmd_set_object_parent,
		simx_cmd_get_array_parameter,
		simx_cmd_set_array_parameter,
		simx_cmd_get_boolean_parameter,
		simx_cmd_set_boolean_parameter,
		simx_cmd_get_integer_parameter,
		simx_cmd_set_integer_parameter,
		simx_cmd_get_floating_parameter,
		simx_cmd_set_floating_parameter,
		simx_cmd_get_string_parameter,
		simx_cmd_read_collision,
		simx_cmd_read_distance,
		simx_cmd_remove_object,
		simx_cmd_remove_ui,
		simx_cmd_get_objects,
		simx_cmd_end_dialog,
		simx_cmd_get_dialog_input,
		simx_cmd_get_dialog_result,
		simx_cmd_copy_paste_objects,
		simx_cmd_set_object_selection,
		simx_cmd_spotcanbeused1,
		simx_cmd_spotcanbeused2,
		simx_cmd_spotcanbeused3,
		simx_cmd_spotcanbeused4,
		simx_cmd_get_model_property,
		simx_cmd_set_model_property,
		simx_cmd_get_object_velocity,
		simx_cmd_remove_model,

		simx_cmd4bytes_custom_start	=0x001800,

		simx_cmd8bytes_start			=0x002000, 
		/* from here on, commands are also identified by additional 8 bytes */
		simx_cmd_get_ui_slider,
		simx_cmd_set_ui_slider,
		simx_cmd_get_ui_event_button,
		simx_cmd_get_ui_button_property,
		simx_cmd_set_ui_button_property,
		simx_cmd_set_ui_button_label,
		simx_cmd_get_object_float_parameter,
		simx_cmd_get_object_int_parameter,
		simx_cmd_set_object_float_parameter,
		simx_cmd_set_object_int_parameter,
		simx_cmd_get_object_child,
		simx_cmd_get_object_group_data,
		simx_cmd_get_object_orientation2,
		simx_cmd_get_object_position2,

		simx_cmd8bytes_custom_start		=0x002800,

		simx_cmd1string_start			=0x003000,
		/* from here on, commands are also identified by one additional string */
		simx_cmd_get_object_handle,
		simx_cmd_load_scene,
		simx_cmd_load_model,
		simx_cmd_transfer_file,
		simx_cmd_load_ui,
		simx_cmd_erase_file,
		simx_cmd_get_ui_handle,
		simx_cmd_add_statusbar_message,
		simx_cmd_aux_console_open,
		simx_cmd_get_collision_handle,
		simx_cmd_get_distance_handle,
		simx_cmd_display_dialog,
		simx_cmd_clear_float_signal,
		simx_cmd_clear_integer_signal,
		simx_cmd_clear_string_signal,
		simx_cmd_get_float_signal,
		simx_cmd_get_integer_signal,
		simx_cmd_get_string_signal,
		simx_cmd_set_float_signal,
		simx_cmd_set_integer_signal,
		simx_cmd_set_string_signal,
		simx_cmd_append_string_signal,
		simx_cmd_write_string_stream=simx_cmd_append_string_signal,
		simx_cmd_get_and_clear_string_signal,
		simx_cmd_read_string_stream,

		simx_cmd1string_custom_start	=0x003800,

		simx_cmdreserved_start			=0x004000,

		simx_cmdmask					=0x00ffff,

		/* Regular operation modes */
		simx_opmode_oneshot				=0x000000,		/* sends command as one chunk. Reply will also come as one chunk. Doesn't wait for the reply. */
		simx_opmode_oneshot_wait		=0x010000,		/* sends command as one chunk. Reply will also come as one chunk. Waits for the reply (_REPLY_WAIT_TIMEOUT_IN_MS is the timeout). */
		simx_opmode_streaming			=0x020000,		/* sends command as one chunk. Command will be stored on the server and always executed (every x ms (as far as possible), where x can be 0-65535. just add x to simx_opmode_streaming). A reply will be sent continuously, each time as one chunk. Doesn't wait for the reply. */
		simx_opmode_continuous			=0x020000,		/* same as simx_opmode_streaming */

		/* Operation modes for heavy data */
		simx_opmode_oneshot_split		=0x030000,		/* sends command as several chunks (max chunk size is x bytes, where x can be _MIN_SPLIT_AMOUNT_IN_BYTES-65535. Just add x to simx_opmode_oneshot_split). Reply will also come as several chunks. Doesn't wait for the reply. */	 
		simx_opmode_streaming_split		=0x040000,		/* sends command as several chunks (max chunk size is x bytes, where x can be _MIN_SPLIT_AMOUNT_IN_BYTES-65535. Just add x to simx_opmode_streaming_split). Command will be stored on the server and always executed. A reply will be sent continuously, each time as several chunks. Doesn't wait for the reply. */
		simx_opmode_continuous_split	=0x040000,		/* same as simx_opmode_streaming_split */

		/* Special operation modes */
		simx_opmode_discontinue			=0x050000,		/* removes and cancels all commands of the given type stored on the client or server side (also streaming commands) */
		simx_opmode_buffer				=0x060000,		/* doesn't send anything, but checks if a reply for the given command is available in the input buffer (i.e. previously received from the server) */
		simx_opmode_remove				=0x070000		/* doesn't send anything and doesn't return any specific value. It just erases a similar command reply in the inbox (to free some memory) */
};

/* Command return codes (bit-coded) */
enum {	simx_return_ok						=0x000000,
		simx_return_novalue_flag			=0x000001,		/* input buffer doesn't contain the specified command. Maybe you forgot to enable data streaming, or streaming hasn't started yet */
		simx_return_timeout_flag			=0x000002,		/* command reply not received in time for simx_opmode_oneshot_wait operation mode */
		simx_return_illegal_opmode_flag		=0x000004,		/* command doesn't support the specified operation mode */
		simx_return_remote_error_flag		=0x000008,		/* command caused an error on the server side */
		simx_return_split_progress_flag		=0x000010,		/* previous similar command not yet fully processed (applies to simx_opmode_oneshot_split operation modes) */
		simx_return_local_error_flag		=0x000020,		/* command caused an error on the client side */
		simx_return_initialize_error_flag	=0x000040		/* simxStart was not yet called */
};

/* Following only for backward compatibility, but equivalent to above return values */
enum {	simx_error_noerror					=0x000000,
		simx_error_novalue_flag				=0x000001,
		simx_error_timeout_flag				=0x000002,
		simx_error_illegal_opmode_flag		=0x000004,
		simx_error_remote_error_flag		=0x000008,
		simx_error_split_progress_flag		=0x000010,
		simx_error_local_error_flag			=0x000020,
		simx_error_initialize_error_flag	=0x000040
};


/******************************************
*******************************************
ROS API constants:
*******************************************
*******************************************/


/* All stream command codes (followed by operation mode codes) */
enum {	simros_strmcmdnull_start				=0,

		/* from here on, commands are only identified by their code */
		simros_strmcmd_get_object_selection,
		simros_strmcmd_get_info, // do not use. Is streamed anyway with topic name "info"

		simros_strmcmdnull_subscriber_start			=0x000800, 
		simros_strmcmd_add_status_bar_message,
		simros_strmcmd_set_object_selection,
		simros_strmcmd_set_joint_state,

		
		simros_strmcmdint_start			=0x001000, 
		/* from here on, commands are also identified by 1 additional int */
		simros_strmcmd_get_array_parameter,
		simros_strmcmd_get_boolean_parameter,
		simros_strmcmd_get_dialog_result,
		simros_strmcmd_get_floating_parameter,
		simros_strmcmd_get_integer_parameter,
		simros_strmcmd_get_joint_state,
		simros_strmcmd_get_object_parent,
		simros_strmcmd_get_objects,
		simros_strmcmd_get_string_parameter,
		simros_strmcmd_get_ui_event_button,
		simros_strmcmd_get_vision_sensor_depth_buffer,
		simros_strmcmd_get_vision_sensor_image,
		simros_strmcmd_read_collision,
		simros_strmcmd_read_distance,
		simros_strmcmd_read_force_sensor,
		simros_strmcmd_read_proximity_sensor,
		simros_strmcmd_read_vision_sensor,
		simros_strmcmd_get_vision_sensor_info,
		simros_strmcmd_get_range_finder_data,
		simros_strmcmd_get_laser_scanner_data,
		simros_strmcmd_get_odom_data,
		simros_strmcmd_get_depth_sensor_data,
		simros_strmcmd_get_tracks_vel,
                simros_strmcmd_get_transform_remapped, 

		simros_strmcmdint_subscriber_start			=0x001800, 
		simros_strmcmd_auxiliary_console_print,
		simros_strmcmd_set_array_parameter,
		simros_strmcmd_set_boolean_parameter,
		simros_strmcmd_set_floating_parameter,
		simros_strmcmd_set_integer_parameter,
		simros_strmcmd_set_joint_force,
		simros_strmcmd_set_joint_position,
		simros_strmcmd_set_joint_target_position,
		simros_strmcmd_set_joint_target_velocity,
		simros_strmcmd_set_vision_sensor_image,
		simros_strmcmd_set_joy_sensor, 
		simros_strmcmd_set_twist_command,
		simros_strmcmd_set_tracks_vel_command,

		
		simros_strmcmdintint_start			=0x002000, 
		/* from here on, commands are also identified by 2 additional ints */
		simros_strmcmd_get_object_pose,
		simros_strmcmd_get_object_float_parameter,
		simros_strmcmd_get_object_int_parameter,
		simros_strmcmd_get_ui_button_property,
		simros_strmcmd_get_ui_slider,
		simros_strmcmd_get_transform,
		simros_strmcmd_get_object_group_data,

		simros_strmcmdintint_subscriber_start			=0x002800, 
		simros_strmcmd_set_object_float_parameter,
		simros_strmcmd_set_object_int_parameter,
		simros_strmcmd_set_object_pose,
		simros_strmcmd_set_object_position,
		simros_strmcmd_set_object_quaternion,
		simros_strmcmd_set_ui_button_label,
		simros_strmcmd_set_ui_button_property,
		simros_strmcmd_set_ui_slider,


		simros_strmcmdstring_start			=0x003000,
		/* from here on, commands are also identified by one additional string */
		simros_strmcmd_get_float_signal,
		simros_strmcmd_get_integer_signal,
		simros_strmcmd_get_string_signal,
		simros_strmcmd_reserved1,
		simros_strmcmd_get_and_clear_string_signal,

		simros_strmcmdstring_subscriber_start			=0x003800, 
		simros_strmcmd_clear_float_signal,
		simros_strmcmd_clear_integer_signal,
		simros_strmcmd_clear_string_signal,
		simros_strmcmd_set_float_signal,
		simros_strmcmd_set_integer_signal,
		simros_strmcmd_set_string_signal,
		simros_strmcmd_reserved2,
		simros_strmcmd_append_string_signal,
		simros_strmcmd_set_joint_trajectory,

		simros_strmcmdintstring_start			=0x004000,
		/* from here on, commands are also identified by one additional int and one additional string */
		simros_strmcmd_get_twist_status,
                                
		simros_strmcmdintstring_subscriber_start			=0x004800, 

		simros_strmcmdreserved_start			=0x005000,

};

#endif /* !defined(V_REPCONST_INCLUDED_) */
