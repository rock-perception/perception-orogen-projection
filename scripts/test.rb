require 'vizkit'
require 'rock/bundle'
require 'readline'

Bundles.initialize
Bundles.transformer.load_conf(Bundles.find_file('config', 'transforms_scripts.rb'))

Bundles.run "projection::ColorizePointcloud" => "colorize", "output" => nil do
    Orocos.name_service << Orocos::CORBA::NameService.new("192.168.56.101")
    tilt_scan = Orocos.name_service.get "tilt_scan_front"
    camera = Orocos.name_service.get "camera"

    col = Orocos.name_service.get "colorize"
    col.transformer_max_latency = 2.0
    col.pointcloud_frame = "body"
    col.camera_frame = "camera_object"
    col.output_ply = "/tmp/objects.ply"
    Bundles.transformer.setup( col )

    tilt_scan.pointcloud.connect_to col.points
    camera.frame.connect_to col.camera

    col.configure
    col.start

    # wait for finish
    Readline.readline "Press RETURN to exit."
end

