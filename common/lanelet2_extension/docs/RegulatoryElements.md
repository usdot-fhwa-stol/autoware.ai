# Regulatory Elements for CARMA

To better support dynamic updates and complex situations, CARMA expands on the standard Lanelet2 regulatory element set. The new regulatory element class definitions can be found [here](../include/lanelet2_extension/regulatory_elements) along with a new TrafficRules object which interprets them in place of the Lanelet2 tagging specification.

## Digital Minimum Gap

Minimum Gap can be set dynamically either through a V2V or V2X communications service. It helps to keep a safety following distance between ADS vehicles within an active geofence.

### Parameters
| **Role** | **Possible Type** | **description**                |
|-------------|--------------|----------------------------------|
| **refers**    | **Lanelet, Area**    | The region of the roadway this minimum gap applies to |

### Custom Attributes

| **Key** | **Value Type** | **description**                |
|-------------|--------------|----------------------------------|
| **subtype** | **digital_minimum_gap**    | Subtype name |
| **mingap** | **double**    | The minimum gap to set. In an osm file units must be meters |
| **participant:XXX** | **yes/no**    | The participant type this applies to |

#### Note on participant tags

To support multiple types of participants a new attribute should be added for each desired type and the value field set to "yes". So a minimum gap which applied to vehicles will have participant:vehicle set to yes. There is no need to explicitally mark participants as no, this will be implied. The lanelet2 participant heirarchy found [here](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/doc/LaneletAndAreaTagging.md#overriding) is supported.

### OSM XML Example

```(xml)
<!-- Lanelet -->
<relation id="1349" visible="true" version="1">
  <member type="way" ref="1347" role="left" />
  <member type="way" ref="1348" role="right" />
  <tag k="location" v="urban" />
  <tag k="subtype" v="road" />
  <tag k="type" v="lanelet" />

  <!-- Minimum gap -->
  <member type='relation' ref='45217' role='regulatory_element' />
</relation>

<!-- Regulatory Minimum gap -->
<relation id='45217' visible='true' version='1'>
  <member type='lanelet' ref='1349' role='refers' />
  <tag k='mingap' v='13' /> <!-- Minimum gap value is of unit meter-->
  <tag k='subtype' v='digital_minimum_gap' />
  <tag k='type' v='regulatory_element' />
  <tag k='participant:vehicle' v='yes' />
</relation>

```

## Digital Speed Limit

Represents a speed limit which can be set dynamically either through a V2X communications service or other mechanism. In a standard use case a digital speed limit would be expected to have precedence over a speed limit from a traffic sign which is still a supported speed limit mechanism.

### Parameters

| **Role** | **Possible Type** | **description**                |
|-------------|--------------|----------------------------------|
| **refers**    | **Lanelet, Area**    | The region of the roadway this speed limit applies to |

### Custom Attributes

| **Key** | **Value Type** | **description**                |
|-------------|--------------|----------------------------------|
| **subtype** | **digital_speed_limit**    | Subtype name |
| **limit** | **Velocity**    | The speed limit to set. In an osm file units must be one of the following m/s, mps, km/h, kmh, m/h, mph |
| **participant:XXX** | **yes/no**    | The participant type this applies to |

#### Note on participant tags

To support multiple types of participants a new attribute should be added for each desired type and the value field set to "yes". So a speed limit which applied to both vehicles and pedestrians will have participant:vehicle and participant:pedestrian set to yes. There is no need to explicitally mark participants as no, this will be implied. The lanelet2 participant heirarchy found [here](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/doc/LaneletAndAreaTagging.md#overriding) is supported.

### OSM XML Example

```(xml)
<!-- Lanelet -->
<relation id="1349" visible="true" version="1">
  <member type="way" ref="1347" role="left" />
  <member type="way" ref="1348" role="right" />
  <tag k="location" v="urban" />
  <tag k="subtype" v="road" />
  <tag k="type" v="lanelet" />

  <!-- Speed Limit -->
  <member type='relation' ref='45218' role='regulatory_element' />
</relation>

<!-- Regulatory speed limit -->
<relation id='45218' visible='true' version='1'>
  <member type='lanelet' ref='1349' role='refers' />
  <tag k='limit' v='5 mph' /> <!-- Speed limit value must have units one of the following   m/s, mps, km/h, kmh, m/h, mph -->
  <tag k='subtype' v='digital_speed_limit' />
  <tag k='type' v='regulatory_element' />
  <tag k='participant:vehicle' v='yes' />
</relation>

```

## Direction Of Travel

A direction of travel regulation defines if a lanelet is One Way or Bi-Directional. Effectively replaces lanelet2's one_way tag

### Parameters

| **Role** | **Possible Type** | **description**                |
|-------------|--------------|----------------------------------|
| **refers**    | **Lanelet**    | The lanelets this regulation applies to |

### Custom Attributes

| **Key** | **Value Type** | **description**                |
|-------------|--------------|----------------------------------|
| **subtype** | **direction_of_travel**    | Subtype name |
| **direction** | **one_way/bi_directional**    | The directions allowed by this regulation |
| **participant:XXX** | **yes/no**    | The participant type this applies to |

#### Note on participant tags

To support multiple types of participants a new attribute should be added for each desired type and the value field set to "yes". So a direction of travel which applied to both vehicles and pedestrians will have participant:vehicle and participant:pedestrian set to yes. There is no need to explicitally mark participants as no, this will be implied. The lanelet2 participant heirarchy found [here](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/doc/LaneletAndAreaTagging.md#overriding) is supported.

### OSM XML Example

```(xml)
<!-- Lanelet -->
<relation id="1349" visible="true" version="1">
  <member type="way" ref="1347" role="left" />
  <member type="way" ref="1348" role="right" />
  <tag k="location" v="urban" />
  <tag k="subtype" v="road" />
  <tag k="type" v="lanelet" />

  <!-- Direction Of Travel -->
  <member type='relation' ref='45220' role='regulatory_element' />
</relation>

<!-- Regulatory Direction Of Travel -->
<relation id='45220' visible='true' version='1'>
  <member type='lanelet' ref='1349' role='refers' />
  <tag k='subtype' v='direction_of_travel' />
  <tag k='type' v='regulatory_element' />
  <tag k='direction' v='one_way' /> <!-- One Way -->
  <tag k='participant:vehicle' v='yes' />
</relation>

```

## Region Access Rule

Represents an access restriction for a lanelet or area. An example would be that only high occupancy vehicles are allowed in a given lane or a bike only lane. Effectively replaces lanelet2's participant overriding behavior.

### Parameters

| **Role** | **Possible Type** | **description**                |
|-------------|--------------|----------------------------------|
| **refers**    | **Lanelet, Area**    | The lanelets or areas this regulation applies to |

### Custom Attributes

| **Key** | **Value Type** | **description**                |
|-------------|--------------|----------------------------------|
| **subtype** | **region_access_rule**    | Subtype name |
| **participant:XXX** | **yes/no**    | The participant type this applies to |

#### Note on participant tags

To support multiple types of participants a new attribute should be added for each desired type and the value field set to "yes". So a region access rule which applied to both vehicles and pedestrians will have participant:vehicle and participant:pedestrian set to yes. There is no need to explicitally mark participants as no, this will be implied. The lanelet2 participant heirarchy found [here](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/doc/LaneletAndAreaTagging.md#overriding) is supported.

### OSM XML Example

```(xml)
<!-- Lanelet -->
<relation id="1349" visible="true" version="1">
  <member type="way" ref="1347" role="left" />
  <member type="way" ref="1348" role="right" />
  <tag k="location" v="urban" />
  <tag k="subtype" v="road" />
  <tag k="type" v="lanelet" />

  <!-- Access Rule -->
  <member type='relation' ref='45219' role='regulatory_element' />
</relation>

<!-- Regulatory Region Access Rule -->
<relation id='45219' visible='true' version='1'>
  <member type='lanelet' ref='1349' role='refers' />
  <tag k='subtype' v='region_access_rule' />
  <tag k='type' v='regulatory_element' />
  <tag k='participant:vehicle:car' v='yes' /> <!-- Allow cars but not trucks -->
</relation>

```

## Passing Control Line

Represents an access restriction in the form of a line laying on the roadway. Restricts whether a given participant can cross the line from the left or right. General usage is as lane boundaries.

A PassingControlLine is created from a list of contiguous LineString3d and participants who are allowed to cross from the left or right. If the control line is representing a lane boundary, each LineString3d parameter should exactly match a right or left bound of an adjacent lanelet. In this fashion, a single regulatory element can represent the lane change restrictions of multiple lanelets while still allowing each lanelet to be associated individually.

The left an right nature of the control line is defined by the non-inverted view of the linestrings which compose it.

### Parameters

| **Role** | **Possible Type** | **description**                |
|-------------|--------------|----------------------------------|
| **ref_line**    | **LineString3d**    | The linestrings which define the geometry of this control line. Must be contiguous |

### Custom Attributes

| **Key** | **Value Type** | **description**                |
|-------------|--------------|----------------------------------|
| **subtype** | **passing_control_line**    | Subtype name |
| **participant:XXX** | **from_left/from_right/from_both**    | The participant type this passing control line applies to |

#### Note on participant tags

To support multiple types of participants a new attribute should be added for each desired type and the value field set to "from_left", "from_right", or "from_both". So a control line which can be cross both directions by both vehicles and pedestrians will have participant:vehicle and participant:pedestrian set to "from_both". There is no need to explicitally mark participants as not passable, this will be implied. The lanelet2 participant heirarchy found [here](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/doc/LaneletAndAreaTagging.md#overriding) is supported.

### OSM XML Example

```(xml)
<!-- Lanelet -->
<relation id="1349" visible="true" version="1">
  <member type="way" ref="1347" role="left" />
  <member type="way" ref="1348" role="right" />
  <tag k="location" v="urban" />
  <tag k="subtype" v="road" />
  <tag k="type" v="lanelet" />

  <!-- Control Line Left -->
  <member type='relation' ref='45221' role='regulatory_element' />
  <!-- Control Line Right -->
  <member type='relation' ref='45222' role='regulatory_element' />
</relation>

<!-- Regulatory Control Line Left -->
<relation id='45221' visible='true' version='1'>
  <member type="way" ref="1347" role="ref_line" />
  <tag k='subtype' v='passing_control_line' />
  <tag k='type' v='regulatory_element' />
  <tag k='participant:vehicle' v='from_both' /> <!-- Both ways -->
</relation>

<!-- Regulatory Control Line Right -->
<relation id='45222' visible='true' version='1'>
  <member type="way" ref="1348" role="ref_line" />
  <tag k='subtype' v='passing_control_line' />
  <tag k='type' v='regulatory_element' />
  <!-- No crossing -->
</relation>

```

## Stop Rule

Represents a virtual stop and wait line horizontally laying on the roadway. It indicates whether a given participant 
should stop and wait momentarily before passing the line. General usage is as a stop line that is not represented by an actual
physical roadway object. By default, it does not apply to all objects except the ones in participant list.

A StopRule is created from a list of contiguous LineString3d and participants who should stop and wait before crossing.
The object is agnostic to the line's invertedness.

### Parameters

| **Role** | **Possible Type** | **description**                |
|-------------|--------------|----------------------------------|
| **ref_line**    | **LineString3d**    | The linestrings which define the geometry of this stop line. Must be contiguous |

### Custom Attributes

| **Key** | **Value Type** | **description**                |
|-------------|--------------|----------------------------------|
| **subtype** | **stop_rule**    | Subtype name |
| **participant:XXX** | **yes/no**    | The participant type this stop rule applies to |

#### Note on participant tags

To support multiple types of participants a new attribute should be added for each desired type and the value field set to "yes", or "no". There is no need to explicitly mark participants as not passable, this will be implied. The lanelet2 participant heirarchy found [here](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/doc/LaneletAndAreaTagging.md#overriding) is supported.

### OSM XML Example

```(xml)
<!-- Lanelet -->
<relation id="1349" visible="true" version="1">
  <member type="way" ref="1347" role="left" />
  <member type="way" ref="1348" role="right" />
  <tag k="location" v="urban" />
  <tag k="subtype" v="road" />
  <tag k="type" v="lanelet" />

  <!-- Stop Rule -->
  <member type='relation' ref='45221' role='regulatory_element' />
</relation>

<!-- Virtual Linestring for Stop Rule>
<way id="1349" visible="false" version="1">
  <nd ref="1339" />
  <nd ref="1342" />
  <tag k="subtype" v="-" />
  <tag k="type" v="virtual" />
</way>

<!-- Regulatory Stop Rule -->
<relation id='45221' visible='true' version='1'>
  <member type="way" ref="1349" role="ref_line" /> <!-- Horizontal linestring representing the stop line -->
  <tag k='subtype' v='stop_rule' />
  <tag k='type' v='regulatory_element' />
  <tag k='participant:vehicle' v='yes' /> <!-- Allow cars but not trucks -->
</relation>

```
## Carma Traffic Signal

Represents a virtual traffic signal with known fixed signal phases. It is virtual because it corresponds to single signal group ID provided by a SAE J2735 SPaT message.
Normally the traffic signal timing information is provided by SAE J2735 SPaT messages although alternative data sources can be supported.

A CarmaTrafficSignal is created from a stop_line (LineString3d) and a list of exit_lanelets (Lanelet). stop_line is where the vehicle should stop and wait before crossing and exit_lanelets represent which direction the vehicle can go based on this traffic signal.

### Parameters

| **Role** | **Possible Type** | **description**                |
|-------------|--------------|----------------------------------|
| **ref_line**    | **LineString3d**    | The linestring which define the geometry of this stop line. |
| **exit_lanelet**    | **Lanelet**    | A lanelet representing the direction this traffic signal controls |

### Custom Attributes

| **Key** | **Value Type** | **description**                |
|-------------|--------------|----------------------------------|
| **subtype** | **carma_traffic_signal**    | Subtype name |

### Note on the creation/usage of this object
Currently when the object is created, it doesn't have any default values for the signal timers. Therefore, setStates function should be used to set the timers before using predictState ever.

### OSM XML Example

```(xml)
<!-- Entry Lanelet -->
<relation id="1349" visible="true" version="1">
  <member type="way" ref="1347" role="left" />
  <member type="way" ref="1348" role="right" />
  <tag k="location" v="urban" />
  <tag k="subtype" v="road" />
  <tag k="type" v="lanelet" />

  <!-- Carma Traffic Signal -->
  <member type='relation' ref='45221' role='regulatory_element' />
</relation>

<!-- Exit Lanelet -->
<relation id="1350" visible="true" version="1">
  <member type="way" ref="1399" role="left" />
  <member type="way" ref="1398" role="right" />
  <tag k="location" v="urban" />
  <tag k="subtype" v="road" />
  <tag k="type" v="lanelet" />
</relation>

<!-- Virtual Linestring for Stop Line >
<way id="1349" visible="false" version="1">
  <nd ref="1339" />
  <nd ref="1342" />
  <tag k="subtype" v="-" />
  <tag k="type" v="virtual" />
</way>

<!-- Regulatory Carma Traffic Signal -->
<relation id='45221' visible='true' version='1'>
  <member type="way" ref="1349" role="ref_line" /> <!-- Horizontal linestring representing the stop line -->
  <member type='relation' ref="1350" role='exit_lanelet' />
  <tag k='subtype' v='carma_traffic_signal' />
  <tag k='type' v='regulatory_element' />
</relation>

```

## Signalized Intersection

Represents signalized intersection on the road. 

SignalizedIntersection consists of ENTRY, EXIT, INTERIOR lanelets that are saved as parameters. 
Although this class doesn't manage it, entry lanelets are expected to have CarmaTrafficSignal class to represent the traffic light.
Therefore, this class can be understood as merely recording of which lanelets are part of the intersection, but traffic signal timer
or which entry correlates to which exit information is handled by each CarmaTrafficSignal object itself.

### Parameters

| **Role** | **Possible Type** | **description**                |
|-------------|--------------|----------------------------------|
| **intersection_entry**    | **Lanelet**    | A lanelet representing the entry of this intersection|
| **intersection_exit**    | **Lanelet**    | A lanelet representing the exit of this intersection|
| **intersection_interior**    | **Lanelet**    |  A lanelet representing the interior of this intersection|

### Custom Attributes

| **Key** | **Value Type** | **description**                |
|-------------|--------------|----------------------------------|
| **subtype** | **signalized_intersection**    | Subtype name |

### OSM XML Example

```(xml)
<!-- Entry Lanelets -->
  <relation id="1349" visible="true" version="1">
    <member type="way" ref="1347" role="left" />
    <member type="way" ref="1348" role="right" />
    <tag k="dynamic" v="no" />
    <tag k="location" v="urban" />
    <tag k="one_way" v="yes" />
    <tag k="subtype" v="road" />
    <tag k="type" v="lanelet" />

    <!-- Signalized Intersection -->
    <member type='relation' ref='45219' role='regulatory_element' />
  </relation>
  <relation id="1352" visible="true" version="1">
    <member type="way" ref="1347" role="left" />
    <member type="way" ref="1348" role="right" />
    <tag k="dynamic" v="no" />
    <tag k="location" v="urban" />
    <tag k="one_way" v="yes" />
    <tag k="subtype" v="road" />
    <tag k="type" v="lanelet" />

    <!-- Signalized Intersection -->
    <member type='relation' ref='45219' role='regulatory_element' />
  </relation>

  <!-- Exit Lanelet -->
  <relation id="1350" visible="true" version="1">
    <member type="way" ref="1347" role="left" />
    <member type="way" ref="1348" role="right" />
    <tag k="dynamic" v="no" />
    <tag k="location" v="urban" />
    <tag k="one_way" v="yes" />
    <tag k="subtype" v="road" />
    <tag k="type" v="lanelet" />
  </relation>

  <!-- Interior Lanelet -->
  <relation id="1351" visible="true" version="1">
    <member type="way" ref="1347" role="left" />
    <member type="way" ref="1348" role="right" />
    <tag k="dynamic" v="no" />
    <tag k="location" v="urban" />
    <tag k="one_way" v="yes" />
    <tag k="subtype" v="road" />
    <tag k="type" v="lanelet" />
  </relation>

  <!-- Regulatory Signalized Intersection Rule -->
  <relation id='45219' visible='true' version='1'>
    <member type='relation' ref='1349' role='intersection_entry' />
    <member type='relation' ref='1350' role='intersection_exit' />
    <member type='relation' ref='1351' role='intersection_interior' />
    <member type='relation' ref='1352' role='intersection_entry' />
    <tag k='subtype' v='signalized_intersection' />
    <tag k='type' v='regulatory_element' />
  </relation>

```