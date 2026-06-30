<!--
  Notes for whoever edits this next:
   - Raw <video>/<figure> HTML works natively here (no \htmlonly needed).
   - Media in blog_media/ is copied to the built site automatically by MkDocs.
   - This theme has no fenced-code extension, so the BibTeX block below is an
     indented (4-space) code block on purpose -- keep it indented.
-->

# Multi-Arm Asynchronous Motion Planning for Manipulation Tasks

<div class="blog-meta">
  <strong>Authors:</strong> Sadra Sadraddini, Ramy Yammine, David Bambrick, Robert Truax, Jackie Videira, Alan Zhou, Bob El-Masri, Brian Krieger, Rana Odabas, Brook Stevens, and Peter Stone &nbsp;&middot;&nbsp;
  <strong>Date:</strong> June 2026 &nbsp;&middot;&nbsp;
  <strong>Paper:</strong> <span style="color:#888">preprint coming soon</span> &nbsp;&middot;&nbsp;
  <strong>Code:</strong> <a href="https://github.com/SonyResearch/planning_service">GitHub</a>
</div>

[TOC]

## TL;DR

We plan coordinated motions for multi-arm robots that operate **asynchronously**:
each arm can replan *while it is still moving* — individually or as a group —
instead of waiting for the others. To produce high-quality motions in the
high-dimensional configuration spaces these systems live in, we pair
sampling-based planning with trajectory optimization over **Graphs of Convex
Sets (GCS)** that are built from prior planning experience and continuously
refined as the system plans more. On a dual-arm **food-plating** system, this
yields substantially higher-quality coordinated trajectories than classical
sampling-based planners, and running the arms asynchronously significantly
increases throughput by 34.47%.

<figure class="blog-figure">
  <iframe
    width="720"
    height="405"
    src="https://www.youtube.com/embed/srIV65WXbHo"
    title="Unedited end-to-end plating shoot, with both arms operating asynchronously (shown at 4x speed)."
    frameborder="0"
    allow="accelerometer; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen>
</iframe>
  <figcaption><strong>Figure 1.</strong> Unedited end-to-end plating shoot, with
  both arms operating asynchronously (shown at 4&times; speed).</figcaption>
</figure>

<figure class="blog-figure">
<iframe
    width="720"
    height="405"
    src="https://www.youtube.com/embed/tPLqhaog5sk"
    title="Plating edited from multiple shoots using two different robots highlighting the robot motions"
    frameborder="0"
    allow="accelerometer; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
    allowfullscreen>
</iframe>
  <figcaption><strong>Figure 2.</strong> Plating edited from multiple shoots using two different robots highlighting robot motions (film credit: Kaiden Stevens).</figcaption>
</figure>

## Abstract
> Multi-arm robotic systems enable coordinated manipulation and parallel task execution, offering substantial gains in capability and throughput relative to single-arm robots. Realizing these benefits, however, requires motion planning in high-dimensional configuration spaces while coordinating asynchronous and heterogeneous robot behaviors. We present a whole-system approach that captures both local and global motion requirements, supports coordinating multiple arms asynchronously, and enables all or a subset of arms to replan while in motion. To generate high-quality motions in high-dimensional spaces, we combine sampling-based methods with trajectory optimization over Graphs of Convex Sets (GCS) that are constructed from prior planning experience and are continuously refined to improve future planning performance. We demonstrate the approach on a dual-arm food-plating platform that combines expressive learned manipulation skills with fast, reliable transit motions. Experiments show that the proposed GCS-based planner generates substantially higher-quality coordinated trajectories than classical sampling-based methods and that asynchronous operation significantly improves throughput compared to sequentially moving arms.

## The Problem

Multiple arms sharing a workspace can manipulate cooperatively and execute tasks
in parallel — more capability, more throughput. The cost of those benefits is the
planning problem underneath:

- **High-dimensional configuration spaces.** Every added arm enlarges the joint
  space the planner must search, and coordinated tasks couple the arms so they
  cannot simply be planned in isolation.
- **Asynchronous, heterogeneous behaviors.** Arms run different skills on
  different timelines. A planner that forces them onto a single shared clock
  wastes time waiting on the slowest arm; letting them run freely instead means
  each arm is a *moving obstacle* to the others and must plan against information
  that is constantly going stale.
- **Replanning in motion.** Useful systems need to adapt mid-task — ideally
  letting any arm, or a subset, replan without stopping the whole cell.


## Approach

We take a **whole-system** view of planning: rather than treating each arm in
isolation, the formulation captures both **local** motion requirements (each
arm's own constraints and skills) and **global** ones (how the arms must
coordinate). On top of that, the planner supports asynchronous coordination and
lets all — or just a subset of — arms replan while in motion.

The core of the method generates high-quality motions in high-dimensional spaces
by combining two ingredients:

1. **Sampling-based exploration** to discover feasible motion in the
   high-dimensional configuration space.
2. **Trajectory optimization over Graphs of Convex Sets (GCS).** The GCS is
   **constructed from prior planning experience** and **continuously refined** as
   the system plans, so planning quality and speed improve over time rather than
   starting from scratch on every query.
3. **Asynchronous coordination.** Arms commit to and execute motions on their own
   timelines, with replanning available to any arm or subset mid-task — which is
   what turns the planner's quality into end-to-end throughput.

> **Key insight.** Reusing and continuously refining planning experience as a
> Graph of Convex Sets lets us optimize *high-quality* coordinated trajectories
> in spaces where sampling alone struggles — and decoupling the arms
> asynchronously converts that trajectory quality into real throughput gains.

<!-- TODO: add pipeline diagram, then uncomment this figure.
<figure class="blog-figure">
  <img src="blog_media/architecture.png" alt="Planner pipeline" width="720">
  <figcaption><strong>Figure 2.</strong> Pipeline: sampling-based exploration
  feeds trajectory optimization over a Graph of Convex Sets that is built from,
  and refined by, past planning experience.</figcaption>
</figure>
-->

## The Platform: Dual-Arm Food Plating

We demonstrate the approach on a **dual-arm food-plating** platform that combines
**expressive learned manipulation skills** (the dexterous plating motions) with
**fast, reliable transit motions** (moving between stations).
We present a dual-arm robotic system with the ultimate target of plating a Michelin-three-star
quality dish. The execution speed is critical as the food quality is highly time-sensitive. For example, cooked salmon needs to be plated and served within minutes to preserve its temperature, taste and texture quality.
Additionally, some ingredients require specialized tools that must
be picked up from a holder before use and returned afterward. The plate, tool holders, and
nearby manipulation zones form a densely shared workspace in which independent runs from arms can
collide with one another, requiring a sophisticated motion planner that can replan quickly while
avoiding collision.

## Experiments & Results

### Quantitative results of making the River Plate
<table cellspacing="0" cellpadding="6" class="blog-table">
  <thead>
    <tr>
      <th style="text-align:left;">Metric</th>
      <th>GCS on</th>
      <th>GCS off</th>
    </tr>
  </thead>
  <tbody>
    <tr style="background-color:#f2f2f2;">
      <td style="text-align:left;">Mean runtime (sync)</td>
      <td>158.15 s</td>
      <td>183.87 s</td>
    </tr>
    <tr>
      <td style="text-align:left;">Mean runtime (async)</td>
      <td>109.13 s</td>
      <td>125.47 s</td>
    </tr>
    <tr style="background-color:#f2f2f2;">
      <td style="text-align:left;">Mean reduction (sync - async)</td>
      <td>49.02 s</td>
      <td>57.92 s</td>
    </tr>
    <tr>
      <td style="text-align:left;">Overall speedup</td>
      <td>31.00%</td>
      <td>31.53%</td>
    </tr>
    <tr style="background-color:#f2f2f2;">
      <td style="text-align:left;">Median paired speedup</td>
      <td>35.17%</td>
      <td>32.86%</td>
    </tr>
    <tr>
      <td style="text-align:left;">P10 paired speedup</td>
      <td>23.52%</td>
      <td>26.01%</td>
    </tr>
    <tr style="background-color:#f2f2f2;">
      <td style="text-align:left;">P90 paired speedup</td>
      <td>37.40%</td>
      <td>35.60%</td>
    </tr>
    <tr>
      <td style="text-align:left;">Async faster (paired trials)</td>
      <td>100% (45/45)</td>
      <td>100% (40/40)</td>
    </tr>
  </tbody>
</table>

*Above Table* Sync vs. async timing in simulation-only benchmarking (no real hardware or
vision; task planning and motion time only), comparing planner configurations with
GCS planning enabled (on) versus solely sample-based planning (GCS off). The
GCS-on run used all successful trials (45 paired comparisons). The GCS-off planning
run reflects the updated filtered benchmark set (40 paired comparisons). Overall
speedup is computed as 1 − _μasync/μsync_.

### Qualitative results

Synchronous vs. asynchronous operation (plating the diced veggies), side by side:

<div class="blog-video-row">
  <figure class="blog-figure">
    <video controls width="352" preload="metadata"
           poster="blog_media/veggies_sync_poster.png">
      <source src="blog_media/veggies_sync.mp4" type="video/mp4">
      <a href="blog_media/veggies_sync.mp4">Download.</a>
    </video>
    <figcaption><strong>(a)</strong> Synchronous operation: arms wait on each
    other between steps.</figcaption>
  </figure>
  <figure class="blog-figure">
    <video controls width="352" preload="metadata"
           poster="blog_media/veggies_async_poster.png">
      <source src="blog_media/veggies_async.mp4" type="video/mp4">
      <a href="blog_media/veggies_async.mp4">Download.</a>
    </video>
    <figcaption><strong>(b)</strong> Asynchronous operation: arms replan in
    motion and overlap their work.</figcaption>
  </figure>
</div>

<!-- ## Discussion & Limitations

PLACEHOLDER -->

## Citation

The paper is not published yet. Until it is, a preprint citation (this is an
indented code block — keep the 4-space indent so it renders without a fenced-code
extension):

    @unpublished{yourkey2026multiarm,
      title  = {Multi-Arm Asynchronous Motion Planning for Manipulation Tasks},
      author = {Sadra Sadraddini, Ramy Yammine, David Bambrick, Robert Truax, Jackie Videira, Alan Zhou, Bob El-Masri, Brian Krieger, Rana Odabas, Brook Stevens, and Peter Stone},
      note   = {Preprint. Under review},
      year   = {2026}
    }

When it appears, switch to the published `@inproceedings{...}` form with the
`booktitle` and `year`.
