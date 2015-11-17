# Copyright © Mapotempo, 2013-2015
#
# This file is part of Mapotempo.
#
# Mapotempo is free software. You can redistribute it and/or
# modify since you respect the terms of the GNU Affero General
# Public License as published by the Free Software Foundation,
# either version 3 of the License, or (at your option) any later version.
#
# Mapotempo is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE.  See the Licenses for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with Mapotempo. If not, see:
# <http://www.gnu.org/licenses/agpl.html>
#
require 'json'
require 'tempfile'


get '/' do
  'optimizer-api'
end

post '/0.1/optimize_tsptw' do
  jdata = JSON.parse(params[:data])
  begin
    optim = optimize(jdata['capacity'], jdata['matrix'], jdata['time_window'], jdata['rest_window'], jdata['optimize_time'], jdata['soft_upper_bound'])
    if !optim
      puts "No optim result !"
      halt(500)
    end
    {status: :ok, optim: optim}.to_json
  rescue Exception => e
    puts e
    halt(500, e.to_s)
  end
end


def optimize(capacity, matrix, time_window, rest_window, optimize_time = nil, soft_upper_bound = nil)
  @exec_vroom = settings.optimizer_vroom_exec
  @exec_or_tools = settings.optimizer_or_tools_exec
  @exec_jsprit = settings.optimizer_jsprit_exec
  @tmp_dir = settings.optimizer_tmp_dir
  @time = optimize_time || settings.optimizer_default_time
  @soft_upper_bound = soft_upper_bound || settings.optimizer_soft_upper_bound

  input = Tempfile.new('optimize-route-input', tmpdir=@tmp_dir)
  output = Tempfile.new('optimize-route-output', tmpdir=@tmp_dir)

  begin
    output.close

    constrains = (time_window + rest_window).find{ |tw| !tw[0].nil? || !tw[1].nil? }

    if !constrains
      # Easy, no constraints, run vroom

      input.write("NAME: vroom\n")
      input.write("TYPE: ATSP\n")
      input.write("DIMENSION: #{matrix.size}\n")
      input.write("EDGE_WEIGHT_TYPE: EXPLICIT\n")
      input.write("EDGE_WEIGHT_FORMAT: FULL_MATRIX\n")
      input.write("EDGE_WEIGHT_SECTION\n")
      input.write(matrix.collect{ |a| a.collect(&:first).join(" ") }.join("\n"))
      input.write("\n")
      input.write("EOF\n")

      input.close

      cmd = "#{@exec_vroom} -t -i '#{input.path}' > '#{output.path}'"
      puts cmd
      system(cmd)
      puts $?.exitstatus
      if $?.exitstatus == 0
        result = File.read(output.path)
        result = JSON.parse(result)['tour']
        index = result.index(1)
        if result[(index + 1) % result.size] == result.size
          result.reverse!
          index = result.size - index - 1
        end
        result.rotate(index).collect{ |i| i - 1 }
      end
    else
      if true  #Add constraint to select Jsprit
	      # hard work to do for vroom, run OR-Tools

	      input.write(matrix.size)
	      input.write("\n")
	      input.write(rest_window.size)
	      input.write("\n")
	      input.write(matrix.collect{ |a| a.collect{ |b| b.join(" ") }.join(" ") }.join("\n"))
	      input.write("\n")
	      input.write((time_window + [[0, 2147483647, 0]]).collect{ |a| [a[0] ? a[0]:0, a[1]? a[1]:2147483647, a[2]].join(" ") }.join("\n"))
	      input.write("\n")
	      input.write(rest_window.collect{ |a| [a[0] ? a[0]:0, a[1]? a[1]:2147483647, a[2]].join(" ") }.join("\n"))
	      input.write("\n")

	      input.close

	      cmd = "#{@exec_or_tools} -time_limit_in_ms #{@time} -soft_upper_bound #{@soft_upper_bound} -instance_file '#{input.path}' > '#{output.path}'"
	      puts cmd
	      system(cmd)
	      puts $?.exitstatus
	      if $?.exitstatus == 0
	        result = File.read(output.path)
	        result = result.split("\n")[1..-1]
	        #puts result.inspect
	        result.collect!{ |i| i.split(' ').collect{ |j| Integer(j) } }
			puts result.inspect
			result
	      end

	  else
		  #Try Jsprit

		  input.write(matrix.size)
	      input.write("\n")
	      input.write(rest_window.size)
	      input.write("\n")
	      input.write(matrix.collect{ |a| a.join(" ") }.join("\n"))
	      input.write("\n")
	      input.write((time_window + [[0, 2147483647, 0]]).collect{ |a| [a[0] ? a[0]:0, a[1]? a[1]:2147483647, a[2]].join(" ") }.join("\n"))
	      input.write("\n")
	      input.write(rest_window.collect{ |a| [a[0] ? a[0]:0, a[1]? a[1]:2147483647, a[2]].join(" ") }.join("\n"))
	      input.write("\n")

	      input.close

	      cmd = "java -jar #{@exec_jsprit} -time_limit_in_ms #{@time} -soft_upper_bound #{@soft_upper_bound} -instance_file '#{input.path}' > '#{output.path}'"
	      puts cmd
	      system(cmd)
	      puts $?.exitstatus
	      if $?.exitstatus == 0
	        result = File.read(output.path)
	        result = result.split("\n")[1..-1]
	        #puts result.inspect
	        result.collect!{ |i| i.split(' ').collect{ |j| Integer(j) } }
			puts result.inspect
			result
		  end
      end
    end
  ensure
    input.unlink
    output.unlink
  end
end
