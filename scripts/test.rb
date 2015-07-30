require 'vizkit'
require 'rock/bundle'
require 'readline'
include Orocos

Orocos::CORBA.max_message_size = 80000000
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
	camera = log.usbcam_front
    else
	Orocos.name_service << Orocos::CORBA::NameService.new("192.168.56.101")
	tilt_scan = Orocos.name_service.get "tilt_scan_front"
	camera = Orocos.name_service.get "camera"
    end

    col = Orocos.name_service.get "colorize"

    m = col.pc2Cam # Eigen::MatrixX.new(4,4)

    m.data[0]= -0.462043
    m.data[1]= 0.531109
    m.data[2]= 0.146176
    m.data[3]= 0.307792
    m.data[4 * 1 + 0]= -0.137454
    m.data[4 * 1 + 1]= -0.00995137
    m.data[4 * 1 + 2]= 0.577735
    m.data[4 * 1 + 3]= 0.188583
    m.data[4 * 2 + 0]= -0.00136313
    m.data[4 * 2 + 1]= -0.0000183015
    m.data[4 * 2 + 2]= 0.000409764
    m.data[4 * 2 + 3]= 0.000876501
    m.data[4 * 3 + 0]= 0
    m.data[4 * 3 + 1]= 0
    m.data[4 * 3 + 2]= 0
    m.data[4 * 3 + 3]= 1
    
    
    col.pc2Cam = m
#     col.transformer_max_latency = 10.0
#     col.pointcloud_frame = "body"
#     col.camera_frame = "camera_object"
#     col.output_ply = "/tmp/objects.ply"
#     Bundles.transformer.setup( col )

    col.camera_period = 1
    col.points_period = 0.01
    
    tilt_scan.pointcloud.connect_to col.points
    camera.frame.connect_to col.camera

    col.configure
    col.start
    
    Vizkit.display col.colored_points

    # wait for finish
    if log
	Vizkit.control log
	Vizkit.exec
    else
	Readline.readline "Press RETURN to exit."
    end
end

