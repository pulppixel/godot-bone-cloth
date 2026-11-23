extends Node3D

@export var automatic := true
@export var fire_rate := 8.0
@export var pellets := 1
@export var pellet_spread_deg := 0.0
@export var spread_per_shot_deg := 0.5
@export var spread_max_deg := 3.0
@export var kick := 1.0
@export var flash := true
@export var shot: PackedScene
@export var sound: AudioStream
