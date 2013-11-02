%% This is a simple trial for robot smart collaboration.

:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_cad_parser).

:- owl_parser:owl_parse('/home/mic/workspace/hydro/rosbuild_ws/pkgs/youbot_coll_test/owl/simple-move.owl', false, false, true).
:- rdf_db:rdf_register_ns(move, 'http://www.dei.unipd.it/kr/simple-move.owl#', [keep(true)]).
