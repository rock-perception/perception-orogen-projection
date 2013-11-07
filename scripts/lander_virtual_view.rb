require 'vizkit'
require 'rock/bundle'
require 'readline'

Bundles.initialize
Bundles.transformer.load_conf(Bundles.find_file('config', 'transforms_scripts.rb'))

Bundles.run "projection::VirtualView" => "virtual_view", "output" => nil do
    Orocos.name_service << Orocos::CORBA::NameService.new("10.250.7.92")
    lander_cam_left = Orocos.name_service.get "lander_cam_left"
    lander_cam_right = Orocos.name_service.get "lander_cam_right"

    vv = Orocos.name_service.get "virtual_view"
    vv.transformer_max_latency = 2.0
    vv.width = 1024
    vv.height = 768
    vv.focal_length = 512
    vv.cam1_frame = "lander_cam_left"
    vv.cam2_frame = "lander_cam_right"
    vv.cam3_frame = "lander_plane"
    vv.cam4_frame = "lander_plane"
    vv.plane_frame = "lander_plane"
    vv.virtual_cam_frame = "lander_virtual_cam"
    Bundles.transformer.setup( vv )

    lander_cam_left.frame.connect_to vv.cam1
    lander_cam_right.frame.connect_to vv.cam2

    vv.configure
    vv.start

    # wait for finish
    Readline.readline "Press RETURN to exit."
end

