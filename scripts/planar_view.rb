require 'vizkit'
require 'rock/bundle'
require 'readline'
include Orocos

Orocos::CORBA.max_message_size = 80000000
Bundles.initialize

log = nil
if !ARGV.empty?
    log = Log::Replay.open ARGV
end

Bundles.run "projection::OmnicamPlanar" => "planar", "output" => nil do
    camera = nil
    if log
	camera = log.cam
    else
	Orocos.name_service << Orocos::CORBA::NameService.new("192.168.10.10")
	camera = Orocos.name_service.get "cam"
    end

    planar = Orocos.name_service.get "planar"
    Orocos.conf.apply( planar, ['default'] )

    camera.frame.connect_to planar.omnicam

    planar.configure
    planar.start
    
    Vizkit.display planar.planar_view

    # wait for finish
    if log
	Vizkit.control log
	Vizkit.exec
    else
	Readline.readline "Press RETURN to exit."
    end
end

