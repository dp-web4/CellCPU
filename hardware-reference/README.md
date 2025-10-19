# CellCPU Hardware Reference Designs

**Status**: Reference designs for educational and non-commercial use
**Patent Coverage**: Protected by US Patent 11,380,942 (Distributed battery management with autonomous cell controllers)
**Last Updated**: October 2025

---

## Overview

This directory contains reference hardware designs for implementing the CellCPU concept - an autonomous cell-level battery management controller based on ATtiny45. These designs demonstrate one possible hardware implementation of the patented distributed battery management architecture.

**Contents**:
- KiCad schematics and PCB layouts
- Bill of Materials (BOM)
- Manufacturing files

---

## Patent Coverage

These hardware designs implement concepts covered by multiple patents in the ModBatt patent portfolio:

### Complete Patent List

**Modular Battery Technologies, Inc. Patent Portfolio**:

1. **US 11,380,942** (PCT/US21/50518) - *High Voltage Battery Module with Series Connected Cells and Internal Relays*
   - Filed: November 2, 2020 | Issued: July 5, 2022
   - Module with series connected cells and relays

2. **US 11,469,470** (PCT/US21/53798) - *Battery Module with Series Connected Cells, Internal Relays and Internal Battery Management System*
   - Filed: January 4, 2021 | Issued: October 11, 2022
   - Cell monitoring/conditioning circuit, PCBAs, methods

3. **US 11,563,241** - *Apparatus and Methods for Removable Battery Module with Internal Relay and Internal Controller*
   - Filed: February 10, 2021 | Issued: December 14, 2022
   - Authentication methods and circuits

4. **US 11,575,270** (PCT/US21/55047) - *Battery Module with Series Connected Cells, Internal Relays and Internal Battery Management System*
   - Filed: February 22, 2021 (CIP) | Issued: February 7, 2023
   - AC coupled comms and methods

5. **US 11,699,817** (PCT/US21/54434) - *Apparatus and Methods for Removable Battery Module with Internal Relay and Internal Controller*
   - Filed: March 31, 2021 | Issued: July 11, 2023
   - System, pack and module controllers, blockchain

6. **US 11,477,027** (PCT/US21/55813) - *Apparatus and Methods for Management of Controlled Objects*
   - Filed: May 11, 2021 | Issued: October 18, 2022
   - Multi-domain management of controlled objects, LCT/blockchain

7. **US 11,936,008** (PCT/US21/60860) - *Electrical Power System with Removable Battery Modules*
   - Filed: November 17, 2021 | Issued: March 19, 2024
   - Dissimilar modules in parallel

8. **US 12,278,913** (PCT/US22/24797) - *Apparatus and Methods for Management of Controlled Objects*
   - Filed: March 31, 2022 | Issued: April 15, 2025
   - Linking of identifiable records

9. **US 11,876,250** (PCT/US22/xxx) - *High Voltage Battery Module with Series Connected Cells and Internal Relays*
   - Filed: May 31, 2022 | Issued: January 16, 2024
   - Dissimilar relays, PLC control bus, linear and PWM modes

10. **US 12,046,722** - *Electrical Power System with Removable Battery Modules*
    - Filed: December 12, 2022 | Issued: July 23, 2024
    - Vehicle and stationary installations having a power system, energizing a bus

11. **US 12,405,309** - *Low Cost Battery Cell Monitoring Circuit*
    - Filed: January 3, 2023 | Issued: September 2, 2025
    - Low cost ASIC

12. **US 12,136,739** - *Low Cost Battery Cell Monitoring Circuit*
    - Filed: December 14, 2023 | Issued: November 5, 2024
    - Low cost ASIC methods

13. **US 12,142,737** - *Electrical Power System with Removable Battery Modules*
    - Filed: December 14, 2023 | Issued: November 12, 2024

14. **US 18/935,265** - *High Voltage Battery Module with 1-Phase and 3-Phase AC Output*
    - Filed: November 1, 2024 | Status: Pending

**Assignee**: Modular Battery Technologies, Inc.
**Inventors**: Dennis Palatov, et al.

### Key Patented Concepts in CellCPU Hardware

- Cell-level autonomous controllers with local intelligence
- Virtual UART communication in daisy-chain topology
- Distributed temperature and voltage monitoring
- Markov blanket interface hiding internal cell complexity
- Low-cost cell monitoring circuits and ASICs
- Authentication methods for cell controllers

---

## License and Usage Terms

### Software License
The **CellCPU firmware** (parent directory) is licensed under **AGPL-3.0-or-later** with a patent grant for software use. See `../LICENSE` for details.

### Hardware Reference Designs
These hardware reference designs are provided under the following terms:

#### ✅ Permitted Uses (Non-Commercial Patent Grant)

You MAY use these designs **without a commercial hardware license** for:

1. **Educational Use**: Academic research, teaching, and learning
2. **Personal/Hobby Use**: Individual projects and experimentation
3. **Research & Development**: Prototyping and testing new concepts
4. **Open Source Development**: Contributing to AGPL-licensed software improvements

**Conditions**:
- No commercial sale or distribution of manufactured hardware
- Acknowledge patent and provide attribution
- Derivative works must include similar non-commercial use notice
- Software modifications must comply with AGPL-3.0-or-later

#### ❌ Restricted Uses (Requires Commercial License)

You MUST obtain a commercial hardware license from Modular Battery Technologies, Inc. for:

1. **Commercial Manufacturing**: Producing hardware for sale
2. **Commercial Products**: Incorporating designs into commercial battery products
3. **Revenue Generation**: Any use where hardware generates revenue directly or indirectly
4. **OEM Integration**: Selling products that include these designs

**Contact**: For commercial licensing inquiries, contact Modular Battery Technologies, Inc.

---

## Technical Scope

These reference designs demonstrate:

- **Cell Controller**: ATtiny45-based autonomous controller per cell
- **Temperature Sensing**: MCP9843 I²C temperature sensor integration
- **Virtual UART**: Software-based serial communication at 20kbps
- **Daisy-Chain Topology**: Up to 94 cells in series connection
- **Voltage Monitoring**: Per-cell voltage measurement architecture
- **Relay Control**: Cell bypass/isolation relay interface

**Voltage Range**: Designed for 13-cell strings (48V nominal LiFePO4)

---

## AS-IS Disclaimer and Limitations

### ⚠️ NO WARRANTY

**THESE HARDWARE REFERENCE DESIGNS ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, TITLE, AND NON-INFRINGEMENT.**

### Specific Disclaimers

1. **No Guarantee of Functionality**: These designs may contain errors, omissions, or may not function as intended
2. **No Safety Certification**: Designs have NOT been certified for safety (UL, CE, etc.)
3. **High Voltage Hazard**: Battery systems operate at dangerous voltages - **risk of electric shock, fire, explosion**
4. **Chemical Hazard**: Lithium battery failures can cause fire, toxic fumes, and explosions
5. **No Professional Engineering Review**: Use at your own risk - consult qualified professionals
6. **Rapidly Evolving**: These designs may be outdated or superseded

### 🚨 Battery Safety Warning

**DANGER**: Lithium-ion and LiFePO4 batteries are potentially hazardous.

- **Thermal runaway** can cause fires and explosions
- **Improper charging** can cause battery damage and fire
- **Short circuits** can cause severe burns and fire
- **Cell imbalance** can reduce lifespan and create safety hazards

**You are solely responsible for:**
- Proper safety engineering and testing
- Compliance with electrical codes and regulations
- Fire suppression and containment systems
- Proper charging and monitoring equipment
- Insurance and liability coverage

---

## Limitation of Liability

**IN NO EVENT SHALL MODULAR BATTERY TECHNOLOGIES, INC., ITS AFFILIATES, OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; BUSINESS INTERRUPTION; PROPERTY DAMAGE; PERSONAL INJURY; OR DEATH) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THESE DESIGNS, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.**

---

## Regulatory Compliance

### Your Responsibility

If you manufacture hardware based on these designs, YOU are responsible for:

1. **Electrical Safety**: UL/CSA/CE compliance as required
2. **FCC/EMC Compliance**: Radio frequency emissions and immunity
3. **Environmental**: RoHS, REACH, WEEE directives
4. **Transportation**: UN38.3 battery shipping requirements (if applicable)
5. **Local Codes**: Electrical codes and building codes in your jurisdiction

### No Compliance Warranty

**These designs have NOT been tested for regulatory compliance.** You must perform your own testing and certification.

---

## Design Files Information

### KiCad Version
Designs created with KiCad EDA (exact version in `.kicad_pro` files)

### File Structure
- `.kicad_sch` - Schematic files (hierarchical sheets)
- `.kicad_pcb` - PCB layout files
- `.csv` / `.ods` - Bill of Materials
- `mfg/` - Manufacturing output files (Gerbers, drill files, pick-and-place)

### Viewing and Editing
1. Download and install KiCad: https://www.kicad.org/
2. Open `.kicad_pro` project file
3. Libraries may need to be remapped on first open

---

## Attribution and Derivative Works

If you create derivative works based on these designs:

1. **Acknowledge the Source**:
   ```
   Based on CellCPU hardware reference designs
   Copyright (c) 2024-2025 Modular Battery Technologies, Inc.
   https://github.com/dp-web4/CellCPU
   ```

2. **Maintain Patent Notice**: Include reference to ModBatt patent portfolio (see Patent Coverage section)

3. **Propagate Non-Commercial Terms**: If distributing derivatives, include similar non-commercial use restrictions

4. **No Endorsement**: Do not imply that your derivative work is endorsed by Modular Battery Technologies, Inc.

---

## Support and Contributions

### No Support Commitment
These reference designs are provided without support. No assistance is guaranteed.

### Community Contributions
Improvements to these reference designs are welcome via pull requests:
- Clearly document changes and reasoning
- Maintain safety-critical design margins
- Include SPDX license headers if adding new files
- Sign-off on contributions (Developer Certificate of Origin)

---

## Relationship to Software License

**Key Distinction**:
- **Software (CellCPU firmware)**: AGPL-3.0-or-later with broad patent grant for software use
- **Hardware (these designs)**: Non-commercial patent grant only; commercial use requires separate license

**Why the Difference?**
- Software improvements benefit the ecosystem when shared (AGPL ensures sharing)
- Hardware manufacturing involves significant capital investment and safety liability
- Patent coverage primarily relates to hardware architecture and topology

**Can I modify the software for use on different hardware?**
- Yes! The AGPL-3.0 software license and patent grant apply to the software
- You can run modified CellCPU software on your own hardware implementations
- Commercial hardware sales still require a hardware patent license

---

## Questions?

**Technical Questions**: Open an issue on GitHub (https://github.com/dp-web4/CellCPU/issues)
**Commercial Licensing**: Contact Modular Battery Technologies, Inc.
**Safety Concerns**: Consult a qualified electrical engineer and safety professional

---

## Legal Summary

| **Aspect** | **Terms** |
|------------|-----------|
| **Software License** | AGPL-3.0-or-later (see `../LICENSE`) |
| **Hardware Designs** | Reference only - non-commercial patent grant |
| **Patents** | 14 US patents (1 pending) - commercial use requires license |
| **Warranty** | NONE - "AS IS" |
| **Liability** | User assumes all risk and liability |
| **Safety** | NOT certified - dangerous high voltage battery application |
| **Commercial Use** | Requires separate commercial license agreement |

---

**© 2024-2025 Modular Battery Technologies, Inc.**
**Patents**: 14 US patents covering battery management systems (see Patent Coverage section)
**Reference designs provided for educational and non-commercial use only**

---

*Last updated: October 2025*
*For latest terms, see https://github.com/dp-web4/CellCPU*
