require 'vizkit'
require 'rock/bundle'
require 'readline'
include Orocos

Bundles.initialize
Bundles.transformer.load_conf(Bundles.find_file('config', 'transforms_scripts.rb'))

log = nil
if !ARGV.empty?
    log = Log::Replay.open ARGV
end

Bundles.run "projection::ColorizePointcloud" => "colorize", "output" => nil do
    tilt_scan = camera = nil
    if log
	tilt_scan = log.tilt_scan_front
	camera = log.camera
    else
	Orocos.name_service << Orocos::CORBA::NameService.new("192.168.56.101")
	tilt_scan = Orocos.name_service.get "tilt_scan_front"
	camera = Orocos.name_service.get "camera"
    end

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
    if log
	Vizkit.control log
	Vizkit.exec
    else
	Readline.readline "Press RETURN to exit."
    end
end

