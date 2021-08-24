# CBSH2-RTC
An optimal solver for Multi-Agent Path Finding.
 

This solver consists of Conflict-Based Search [1] and many of its improvement techniques, including 
* Prioritizing conflicts [2]
* Bypassing conflicts [3]
* High-level admissible heuristics:
    * CG [4] 
    * DG [5]
    * WDG [5]
* Symmetry reasoning techniques:
    * rectangle reasoning [6] and generalized rectangle reasoning [7]
    * target reasoning [8]
    * corridor reasoning [8] and corridor-target reasoning [7]
    * mutex propagation [9]
* Disjoint splitting [10]
 
 
 
Please cite the following paper if you use the code in your published research:  
Jiaoyang Li, Daniel Harabor, Peter J. Stuckey, Hang Ma, Graeme Gange and Sven Koenig.
[Pairwise Symmetry Reasoning for Multi-Agent Path Finding Search](https://doi.org/10.1016/j.artint.2021.103574).
Artifical Intelligence (AIJ), volume 301, pages 103574, 2021.
 
 ## Usage
The code requires the external library BOOST (https://www.boost.org/). 
You can follow the instruction on the BOOST webpage to install it. Or, 
if you are using Ubantu, you can install it simply by
```shell script
sudo apt install libboost-all-dev
``` 

After you installed BOOST and downloaded the source code, go into the directory of the source code and compile it with CMake: 
```shell script
cmake .
make
```

Then, you are able to run the code:
```shell script
./cbs -m random-32-32-20.map -a random-32-32-20-random-1.scen -o test.csv --outputPaths=paths.txt -k 30 -t 60
```

- m: the map file from the MAPF benchmark
- a: the scenario file from the MAPF benchmark
- o: the output file that contains the search statistics
- outputPaths: the output file that contains the paths 
- k: the number of agents
- t: runtime limit (in seconds)

The above command runs the best variant of the code reported in [7] (i.e., using 
prioritizing conflicts,
bypassing conflicts,
WDG heuristics,
target reasoning, and
generalized rectangle and corridor reasoning).

If you want to turn on/off some techniques,
you can find more details and explanations for all parameters with:
```shell script
./cbs --help
```

To test the code on more instances,
you can download the MAPF instances from the MAPF benchmark (https://movingai.com/benchmarks/mapf/index.html).

## License
The code is released under USC – Research License. See license.md for further details.

I would like to thank Han Zhang for providing the code for mutex propagation.

## References

[1] Guni Sharon, Roni Stern, Ariel Felner, and Nathan R. Sturtevant.
Conflict-Based Search for Optimal Multi-Agent Pathfinding.
Artificial Intelligence, 219:40–66, 2015.

[2] Eli Boyarski, Ariel  Felner, Roni Stern, Guni Sharon, David Tolpin, Oded Betzalel, and Solomon Eyal Shimony.
ICBS: Improved Conflict-Based Search Algorithm for Multi-Agent Pathfinding. 
In Proceedings of the International Joint Conference on Artificial Intelligence (IJCAI), pages 740–746, 2015.

[3] Eli Boyarski, Ariel Felner, Guni Sharon, and Roni Stern.
Don't Split, Try to Work It Out: Bypassing Conflicts in Multi-Agent Pathfinding. 
In Proceedings of the International Conference on Automated Planning and Scheduling (ICAPS), pages 47-51, 2015.

[4] Ariel Felner, Jiaoyang Li, Eli Boyarski, Hang Ma, Liron Cohen, T. K. Satish Kumar, and Sven Koenig.
Adding Heuristics to Conflict-Based Search for Multi-Agent Path Finding. 
In Proceedings of the International Conference on Automated Planning and Scheduling (ICAPS), pages 83-87, 2018.

[5] Jiaoyang Li, Ariel Felner, Eli Boyarski, Hang Ma, and Sven Koenig.
Improved Heuristics for Multi-Agent Path Finding with Conflict-Based Search.
In Proceedings of the International Joint Conference on Artificial Intelligence (IJCAI), pages 442-449, 2019.

[6] Jiaoyang Li, Daniel Harabor, Peter J. Stuckey, Hang Ma, and Sven Koenig.
Symmetry-Breaking Constraints for Grid-Based Multi-Agent Path Finding.
In Proceedings of the AAAI Conference on Artificial Intelligence (AAAI), pages 6087-6095, 2019.

[7] Jiaoyang Li, Daniel Harabor, Peter J. Stuckey, and Sven Koenig. 
Pairwise Symmetry Reasoning for Multi-Agent Path Finding Search.
CoRR, abs/2103.07116, 2021.

[8] Jiaoyang Li, Graeme Gange, Daniel Harabor, Peter J. Stuckey, Hang Ma, and Sven Koenig.
New Techniques for Pairwise Symmetry Breaking in Multi-Agent Path Finding.
In Proceedings of the International Conference on Automated Planning and Scheduling (ICAPS), pages 193-201, 2020.

[9] Han Zhang, Jiaoyang Li, Pavel Surynek, Sven Koenig, and T. K. Satish Kumar.
Multi-Agent Path Finding with Mutex Propagation.
In Proceedings of the International Conference on Automated Planning and Scheduling (ICAPS), pages 323-332, 2020.
 
[10] Jiaoyang Li, Daniel Harabor, Peter J. Stuckey, Ariel Felner, Hang Ma, and Sven Koenig.
Disjoint Splitting for Multi-Agent Path Finding with Conflict-Based Search.
In Proceedings of the International Conference on Automated Planning and Scheduling (ICAPS), pages 279-283, 2019.
