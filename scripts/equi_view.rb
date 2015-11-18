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

Bundles.run "projection::OmnicamEquirectangular" => "equi", "output" => nil do
    camera = nil
    if log
	camera = log.cam
    else
	Orocos.name_service << Orocos::CORBA::NameService.new("192.168.10.10")
	camera = Orocos.name_service.get "cam"
    end

    equi = Orocos.name_service.get "equi"
    Orocos.conf.apply( equi, ['default'] )

    camera.frame.connect_to equi.omnicam

    equi.configure
    equi.start
    
    Vizkit.display equi.view

    # wait for finish
    if log
	Vizkit.control log
	Vizkit.exec
    else
	Readline.readline "Press RETURN to exit."
    end
end

