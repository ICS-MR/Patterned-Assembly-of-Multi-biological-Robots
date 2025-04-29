# Patterned Assembly of Multi-biological Robots with Global Input


We propose a collaborative control method for multi-biological magnetic microrobots to achieve complex spatial arrangements. This method overcomes persistent complex and scalability constraints in organoids assembly, enabling deterministic construction of different biological structures with tunable functions.


## 🔥 Updates

* [2025-05] Release code.

## 🔨 Installation

Code was tested under Ubuntu 22.04.

Clone the repo first:

```Bash
git clone https://github.com/ICS-MR/Patterned-Assembly-of-Multi-biological-Robots
cd patternedAssembly
```

 Create the conda env called patternedAssembly  :

```Bash
conda env create -f patternedAssembly_environment.yaml
conda activate patternedAssembly
```


## 💡 Usage


```Bash
 cd PresentationLayer/ 
 python SimulationExperiments.py
```

Then you can see the patterned assembly process of shape T.

You can also choose other experiments and rerun them by changing the parameters.
Open `SimulationExperiments.py` 

`line 33 argument = 'T'`  Parameter argument can be changed to theexperiment mentioned in the note.
`line 37 OperationalSetting = OperationalSettings[1]`  The number in [] can be changed to choose operation. If the number is 0, experiment will start from scratch and it may spend some time. If the number is 1, The pre-computed data will be displayed.

## Citation

coming soon...
