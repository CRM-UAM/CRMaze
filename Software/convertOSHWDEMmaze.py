with open("mazes.txt") as f:
    content = f.readlines()
ind=1
for l in content[1:]:
    if (ind%2) == 0:
        indc=0
        for c in l:
            if indc%4 == 0 and indc <= 52:
                if c == '|' :
                    print("1"),
                else:
                    print("0"),
            elif indc%4 == 1 and indc <= 52:
                print("0"),
            indc+=1
        print("")
    else:
        for i in range(0,53,4):
            print ("1"),
            #print l[i+1:i+4]
            if l[i+1:i+4] == "---":
                print("1"),
            elif (i+3) < 52:
                print("0"),

        print("")

    ind+=1

