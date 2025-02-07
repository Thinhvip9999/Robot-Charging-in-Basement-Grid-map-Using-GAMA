/**
* Name: AlgorithmsOnMap
* Based on the internal skeleton template. 
* Author: hnv
* Tags: 
*/

model AlgorithmsOnMap

global {
	/** Insert the global definitions, variables and actions here */
	string scenario <- "basement map" among: ["random map","basement map"] parameter: true;
	string algorithm <- "A*" among: ["A*", "Dijkstra", "JPS", "BF"] parameter: true;
	int neighborhood_type <- 4 among: [4, 8] parameter: true;
	float obstacle_rate <- 0.2 min: 0.0 max: 0.4 parameter: true;
	int grid_size_height <- 40;
	int grid_size_width <- 40;
	point source; 
	point goal;
	path the_path;
	int robot_step <- 0;
	int num_of_robot_init  <- 1;
	point robot_location;
	bool reach_goal;
	
	int energy_limit <- 400 min: 0 max: 2500 parameter: true;
	list charging_location;
	int shortest_path_length_to_charging_location <- 100000;
	point shortest_charging_location;
	int num_of_charging_pole_init <- 1;
	bool reach_charging_pole;
	bool incharging_process <- false;
	int times_charging_under_50;
	int times_charging_from_50_to_100;
	int times_charging_from_100_to_150;
	int times_charging_from_150_to_200;
	
	int times_path_length_under_50;
	int times_path_length_from_50_to_75;
	int times_path_length_from_75_to_100;
	int times_path_length_above_100;
	
	list list_of_goals;
	list<file> ev_car_images <- [
		file("../includes/images/electric-car-green.png"),
		file("../includes/images/electric-car-blue.png"),
		file("../includes/images/electric-car-red.png"),
		file("../includes/images/electric-car-purple.png")
	];
	int type_of_ev <- rnd(length(ev_car_images)-1);
	
	//change this into path.segment
	list<path> total_path;
	
	float general_size <- 2.0;
	
	list<file> images_button <- [
		file("../includes/images/charger.png"),
		file("../includes/images/bin.png")
	];
	int action_type <- -1;	
	
	init toto{
		if (scenario = "basement map") {
			ask cell {is_obstacle <- false;}
			//load file csv cell.is_obstacle <- true;
			// ask cell {color <- is_obstacle ? #black : #white;}
			file Map2_file <- csv_file("../includes/parking_lot.csv");
			matrix Map2_matrix <- matrix(Map2_file);
			
			loop i from: 0 to: grid_size_height -1 {
				loop j from: 0 to: grid_size_width -1{
					if (int(Map2_matrix[j, i]) = 1) {
						cell[j, i].is_obstacle <- true;
					}
					// Chọn 1 số trong csv tượng trưng cho list of goal
					if (int(Map2_matrix[j, i]) = 2) {
						cell[j, i].is_in_goal_list <- true;
						list_of_goals <+ cell[j, i].location;
					}
				}
			}
			ask cell {
				if(is_obstacle){
					color <- #black;
				} else if (is_in_goal_list){
					color <- #red;
				} else {
					color <- #white;
				}
			}
		}
		source <- (one_of (cell where not each.is_obstacle)).location;
		//Thay đổi điểm goal không còn là tùy ý nữa mà phải là khu đặc biệt có is_goal = true
		goal <- (one_of (cell where each.is_in_goal_list)).location;
		robot_location <- point(source);
		charging_location <- [(cell[19, 5]).location, (cell[2, 39]).location, (cell[38, 18]).location];
		using topology(cell) {
			write("Before: " + length(total_path) + " Initialize stage!");
			the_path <- path_between((cell where not each.is_obstacle), source, goal);
			total_path <+ the_path;
			write("After: " + length(total_path) + " Initialize stage!");
			write("-----------END-----------");
		}
		create robot number: num_of_robot_init;
		loop i from:0 to: length(charging_location) -1 {
			create charging_pole number: num_of_charging_pole_init with: (location: charging_location[i]);
		}
	}
	
	reflex pause_simulation when: (energy_limit = 0) {
		do pause;
	}
	
	reflex move_to_charge when: (reach_goal and energy_limit < 200){
		write("Robot have reach_goal ? " + reach_goal);
		write("Robot have " + energy_limit+ " left before charging");
		reach_goal <- false;
		source <- robot_location;
		shortest_path_length_to_charging_location <- 100000;
		//Chọn charging_location có vị trí ngắn nhất
		loop i from: 0 to: length(charging_location) -1 {
			point check_start <- robot_location;
				shortest_charging_location <- charging_location[i];	
			}
		goal <- shortest_charging_location; 
		using topology(cell){
			write("Before: " + length(total_path) + " Charging stage!");
			the_path <- path_between((cell where not each.is_obstacle), source, goal);
			total_path <+ the_path;
			write("After: " + length(total_path) + " Charging stage!");
			write("-----------END-----------");
			if (length(the_path.vertices) < 50){
				times_path_length_under_50 <- times_path_length_under_50 + 1;
			} else if (length(the_path.vertices) < 75){
				times_path_length_from_50_to_75 <- times_path_length_from_50_to_75 + 1;
			} else if (length(the_path.vertices) < 100){
				times_path_length_from_75_to_100 <- times_path_length_from_75_to_100 + 1;
			} else {
				times_path_length_above_100 <- times_path_length_above_100 + 1;
			}
		}
		reach_charging_pole <- robot_location = goal;
		if (reach_charging_pole){
			if (energy_limit < 50){
				times_charging_under_50 <- times_charging_under_50 + 1;
			} else if (energy_limit < 100) {
				times_charging_from_50_to_100 <- times_charging_from_50_to_100 + 1;
			} else if (energy_limit < 150) {
				times_charging_from_100_to_150 <- times_charging_from_100_to_150 + 1;
			} else {
				times_charging_from_150_to_200 <- times_charging_from_150_to_200 + 1;
			}
			energy_limit <- energy_limit + 200;
		}
	}
	
	reflex check_robot {
		//Lỗi gặp phải khi chạy 1 path nữa trước khi sạc lẽ ra khi chạy đến sạc thì sẽ không xuất hiện 1 path kia nữa -> đổi check sạc lên trước
		reach_goal <- robot_step = int(length(the_path.vertices));
	 	if (reach_goal){
	 		type_of_ev <- rnd(length(ev_car_images)-1);
			source <- goal;
			//Thay đổi điểm goal không còn là tùy ý nữa mà phải là khu đặc biệt có is_goal = true
			goal <- (one_of (cell where each.is_in_goal_list)).location;
			robot_step <- 0;
			robot_location <- point(source);
			
			using topology(cell){
				write("Before: " + length(total_path) + " Normal stage!");
				the_path <- path_between((cell where not each.is_obstacle), source, goal);
				total_path <+ the_path;
				write("After: " + length(total_path) + " Normal stage!");
				write("-----------END-----------");
				if (length(the_path.vertices) < 50){
					times_path_length_under_50 <- times_path_length_under_50 + 1;
				} else if (length(the_path.vertices) < 75){
					times_path_length_from_50_to_75 <- times_path_length_from_50_to_75 + 1;
				} else if (length(the_path.vertices) < 100){
					times_path_length_from_75_to_100 <- times_path_length_from_75_to_100 + 1;
				} else {
					times_path_length_above_100 <- times_path_length_above_100 + 1;
				}
			}
		}
	}
//	reflex compute_path {
//		source <- (one_of (cell where not each.is_obstacle)).location;
//		goal <- (one_of (cell where not each.is_obstacle)).location;
//		
//		using topology(cell) {
//			the_path <- path_between((cell where not each.is_ob stacle), source, goal);	
//		}
//	}
	action activate_act {
		button selected_but <- first(button overlapping (circle(1) at_location #user_location));
		if(selected_but != nil) {
			ask selected_but {
				ask button {bord_col<-#black;}
				if (action_type != id) {
					action_type<-id;
					bord_col<-#red;
				} else {
					action_type<- -1;
				}
				
			}
		}
	}
}

species robot {
	float size <- general_size;
	rgb color <- #blue;
	image_file robot_icon <- image_file("../includes/images/robot.png");
	
	init {
		location <- robot_location;
	}
	
	reflex basic_move {
		location <- the_path.vertices[robot_step];
		robot_step <- robot_step + 1;
		energy_limit <- energy_limit - 1;
	}
		
	aspect base{
		draw circle(size) color: color;
	}
	
	aspect info {
		draw square(size) color: color;
		draw string(energy_limit with_precision 2) size: 3 color: #black;
	}
	
	aspect icon{
		draw robot_icon size: size;
	}
}

species charging_pole {
	float size <- general_size;
	rgb color <- #purple;
	image_file charging_pole_icon <- image_file("../includes/images/charger.png");
	
//	init {
//		location <- one_of(charging_location);
//	}
	
	aspect base{
		draw circle(size) color: color;
	}
	
	aspect icon{
		draw charging_pole_icon size: size;
	}
}

grid cell width: grid_size_width height: grid_size_height neighbors: neighborhood_type optimizer: algorithm {
	bool is_obstacle;
	bool is_in_goal_list;
	rgb color_obstacle <- is_obstacle ? #black : #white;
	rgb color_charging_position <- is_in_goal_list ? #red : #white;
} 

grid button width:2 height:1 
{
	int id <- int(self);
	rgb bord_col<-#black;
	aspect normal {
		draw rectangle(shape.width * 0.8,shape.height * 0.8).contour + (shape.height * 0.01) color: bord_col;
		draw image_file(images_button[id]) size:{shape.width * 0.5,shape.height * 0.5} ;
	}
}

experiment AlgorithmsOnMap type: gui {
	/** Insert here the definition of the input and output of the model */
	output synchronized: true {
		layout horizontal([0.0::4500,1::500, 2::2500, 3::2500]);
		display main_display type: 2d antialias: false {
			grid cell border: #black;
			graphics "elements" {
				if (length(total_path) > 1){
					loop history_path_index from: 0 to: length(total_path)-2 {
						path history_path <- total_path[history_path_index];
						loop b over: history_path.segments{
							draw b color: #blue;
						}
						loop c over: history_path.vertices {
							draw circle(0.1) color: #red border: #blue at: point(c);
						}
					}
				}
				loop v over: the_path.vertices {
					draw triangle(0.5) color: #yellow border: #red at: point(v);
				}
				loop s over: the_path.segments {
					draw s color: #red ;
				}
				draw image_file(ev_car_images[type_of_ev]) size:{shape.width * 0.025,shape.height * 0.025} at: goal;
			}
			species robot aspect: icon;
			species charging_pole aspect: icon;
		}
		display action_button background:#black name:"Tools panel"  type:2d antialias:false {
			species button aspect:normal ;
			event #mouse_down {ask simulation {do activate_act;}}  
		}
		
		display "Robot Infomation" type: 2d {
			chart "Robot Energy Real Time" type: series x_label: "Time frames" memorize: false {
				data "Battery" value: energy_limit color: #green marker: false style: line;
			}
		}
		display "Path Information" type: 2d {
			chart "Path Length Statistics" type: histogram size: {0.5, 1} position: {0,0}
			series_label_position: onchart
			{
				datalist legend: ["Short", "Moderate", "Long", "Very Long"]
				style: bar
				value: [times_path_length_under_50, times_path_length_from_50_to_75, times_path_length_from_75_to_100, times_path_length_above_100]
				color: [#red, #yellow, #green, #darkgreen];
			}
			chart "How Robot Energy Left Before Charging affect battery" type: histogram size: {0.5, 1} position: {0.5, 0}
			series_label_position: onchart
			{
				datalist legend: ["Bad", "Moderate", "Good", "Very Good"]
				style: bar
				value: [times_charging_under_50, times_charging_from_50_to_100, times_charging_from_100_to_150, times_charging_from_150_to_200]
				color: [#red, #yellow, #green, #darkgreen];
			}
		}
	}
}
