# CommonRoad - Autoware Interface (CR2AW)
This interface is used to couple CommonRoad with the planning part of Autoware both in simulation and on 
real vehicles. This enables to use various motion planners using the CommonRoad format within the Autoware software stack 
and rapidly transfer new algorithms from simulation to a real vehicle.

**Our interface in simulation and in a real test drive using the TUM research vehicle [EDGAR](https://arxiv.org/pdf/2309.15492):**

![Real Test Drive and Simulation with our Inteface](assets/readme_image.svg)


**A video of an autonomous test drive with CR2AW:**

<iframe width="560" height="315" src="https://www.youtube.com/embed/Jfp7YR6NYRo?si=ebCYuaewqgh8mhQ-" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

---

## Requirements & Setup
Please consult the README for detailed instructions regarding system requirements and setup.

---

## Citation
If you use our code for research, please cite our [paper](https://mediatum.ub.tum.de/doc/1740269/h9fhalm4tfqbb9abjpd33gwpc.pdf):

```
@inproceedings{Wuersching2024b
  author = {Würsching, Gerald and Mascetta, Tobias and Lin, Yuanfei and Althoff, Matthias},
  title = {Simplifying Sim-to-Real Transfer in Autonomous Driving: Coupling Autoware with the CommonRoad Motion Planning Framework},
  booktitle = {Proc. of the IEEE Intelligent Vehicles Symposium},
  year={2024},
  pages = {tbd},
}
```

---

## Authors
- **Gerald Würsching**: gerald.wuersching[at]tum.de
- **Tobias Mascetta**: tobias.mascetta[at]tum.de
- **Yuanfei Lin**: yuanfei.lin[at]tum.de


---

## Contact Information
- **Release:** 0.2.0
- **Website:** http://commonroad.in.tum.de
- **Email:** commonroad@lists.lrz.de