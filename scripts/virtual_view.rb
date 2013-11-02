require 'vizkit'
require 'rock/bundle'
require 'readline'

Bundles.initialize
Bundles.transformer.load_conf(Bundles.find_file('config', 'transforms_scripts.rb'))

Bundles.run "projection::VirtualView" => "virtual_view", "output" => nil do
    Orocos.name_service << Orocos::CORBA::NameService.new("192.168.56.101")
    camera_obj = Orocos.name_service.get "camera"
    camera_front = Orocos.name_service.get "prosilica_front"

    vv = Orocos.name_service.get "virtual_view"
    vv.transformer_max_latency = 2.0
    vv.cam1_frame = "camera_object"
    vv.cam2_frame = "camera_front"
    vv.cam3_frame = "body"
    vv.cam4_frame = "body"
    Bundles.transformer.setup( vv )

    camera_obj.frame.connect_to vv.cam1
    camera_front.frame.connect_to vv.cam2

    vv.configure
    vv.start

    # wait for finish
    Readline.readline "Press RETURN to exit."
end

