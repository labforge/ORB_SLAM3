file_name="slam_log_file"
file = open(file_name,"r")
file_trimmed = open(file_name+"_trimmed","w+")
prev_line = ""
count = 0
print_cnt = 0
for line in file:
	if(line == prev_line):
		count+=1
	#if(line != prev_line):
	else:
		#print(prev_line,end='')
		file_trimmed.write(prev_line)
		if(count!=0):
			#print(count)
			file_trimmed.write(str(count)+"\n")
		count=0
	prev_line = line

file.close()
file_trimmed.close()
