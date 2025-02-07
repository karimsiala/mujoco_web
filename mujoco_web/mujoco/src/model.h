
#pragma once

#include <emscripten/bind.h>
#include <emscripten/fetch.h>
#include <emscripten/val.h>

#include "mujoco/mujoco.h"

#include <iostream>

using namespace emscripten;

/**
 * @brief Manages loading and accessing MuJoCo models.
 *
 * The `Model` class provides functionality to load MuJoCo models from
 * binary (.mjb) or XML (.xml) files. It encapsulates the `mjModel` pointer
 * and offers methods to access various model parameters. The class handles
 * err checking during model loading and integrates with Emscripten for
 * JavaScript bindings.
 * ## Usage Example:
 *
 * ```cpp
 * Model model("path/to/model.xml");
 * if (!model.ptr()) {
 * // Handle loading err
 * }
 */
class Model {

private:
  /**
   * @brief Cleans up and deletes a MuJoCo model.
   *
   * This function is used to handle errs that occur during the loading of a
   * MuJoCo model. It will delete the model if it was successfully loaded, and
   * print an err message if provided.
   *
   * @param msg An optional err message to print.
   * @param m The MuJoCo model to delete, if it was successfully loaded.
   * @return 0 to indicate the function completed successfully.
   */
  int finish(mjModel *m = NULL) {
    if (m) {
      mj_deleteModel(m);
    }
    if (!error.empty()) {
      std::cout << error << std::endl;
    }
    std::cout << "Model deleted" << std::endl;
    return 0;
  }

public:
  Model() {
    m = nullptr;
    error = "";
  }

  Model(const std::string &filename) {
    error = "";
    const int buffer_size = 1000;
    char error_buffer[buffer_size];
    if (0 == filename.compare(filename.length() - 3, 3, "mjb")) {
      m = mj_loadModel(filename.c_str(), 0);
      if (!m) {
        error = "Error loading mjb file";
        finish(m);
      }
    } else {
      m = mj_loadXML(filename.c_str(), 0, error_buffer, buffer_size);
      if (!m) {
        const auto error_details = std::string{error_buffer};
        error = "Error loading xml file: " +
                (error_details.empty() ? "reason unknown" : error_details);
        finish(m);
      }
    }
  }

  mjModel *ptr() { return m; }
  mjModel getVal() { return *m; }
  mjOption getOptions() { return (*m).opt; }
  std::string getError() { return error; }

  /**
   * Free the memory allocated for the MuJoCo model.
   */
  void free() { return mju_free(m); }

  // clang-format off

  // DEFINITIONS FROM MJMODEL.H

  int nq() const { return m->nq; }
  int nv() const { return m->nv; }
  int nu() const { return m->nu; }
  int na() const { return m->na; }
  int nbody() const { return m->nbody; }
  int nbvh() const { return m->nbvh; }
  int nbvhstatic() const { return m->nbvhstatic; }
  int nbvhdynamic() const { return m->nbvhdynamic; }
  int njnt() const { return m->njnt; }
  int ngeom() const { return m->ngeom; }
  int nsite() const { return m->nsite; }
  int ncam() const { return m->ncam; }
  int nlight() const { return m->nlight; }
  int nflex() const { return m->nflex; }
  int nflexvert() const { return m->nflexvert; }
  int nflexedge() const { return m->nflexedge; }
  int nflexelem() const { return m->nflexelem; }
  int nflexelemdata() const { return m->nflexelemdata; }
  int nflexelemedge() const { return m->nflexelemedge; }
  int nflexshelldata() const { return m->nflexshelldata; }
  int nflexevpair() const { return m->nflexevpair; }
  int nflextexcoord() const { return m->nflextexcoord; }
  int nmesh() const { return m->nmesh; }
  int nmeshvert() const { return m->nmeshvert; }
  int nmeshnormal() const { return m->nmeshnormal; }
  int nmeshtexcoord() const { return m->nmeshtexcoord; }
  int nmeshface() const { return m->nmeshface; }
  int nmeshgraph() const { return m->nmeshgraph; }
  int nskin() const { return m->nskin; }
  int nskinvert() const { return m->nskinvert; }
  int nskintexvert() const { return m->nskintexvert; }
  int nskinface() const { return m->nskinface; }
  int nskinbone() const { return m->nskinbone; }
  int nskinbonevert() const { return m->nskinbonevert; }
  int nhfield() const { return m->nhfield; }
  int nhfielddata() const { return m->nhfielddata; }
  int ntex() const { return m->ntex; }
  int ntexdata() const { return m->ntexdata; }
  int nmat() const { return m->nmat; }
  int npair() const { return m->npair; }
  int nexclude() const { return m->nexclude; }
  int neq() const { return m->neq; }
  int ntendon() const { return m->ntendon; }
  int nwrap() const { return m->nwrap; }
  int nsensor() const { return m->nsensor; }
  int nnumeric() const { return m->nnumeric; }
  int nnumericdata() const { return m->nnumericdata; }
  int ntext() const { return m->ntext; }
  int ntextdata() const { return m->ntextdata; }
  int ntuple() const { return m->ntuple; }
  int ntupledata() const { return m->ntupledata; }
  int nkey() const { return m->nkey; }
  int nmocap() const { return m->nmocap; }
  int nplugin() const { return m->nplugin; }
  int npluginattr() const { return m->npluginattr; }
  int nuser_body() const { return m->nuser_body; }
  int nuser_jnt() const { return m->nuser_jnt; }
  int nuser_geom() const { return m->nuser_geom; }
  int nuser_site() const { return m->nuser_site; }
  int nuser_cam() const { return m->nuser_cam; }
  int nuser_tendon() const { return m->nuser_tendon; }
  int nuser_actuator() const { return m->nuser_actuator; }
  int nuser_sensor() const { return m->nuser_sensor; }
  int nnames() const { return m->nnames; }
  int nnames_map() const { return m->nnames_map; }
  int npaths() const { return m->npaths; }
  int nM() const { return m->nM; }
  int nB() const { return m->nB; }
  int nC() const { return m->nC; }
  int nD() const { return m->nD; }
  int ntree() const { return m->ntree; }
  int ngravcomp() const { return m->ngravcomp; }
  int nemax() const { return m->nemax; }
  int njmax() const { return m->njmax; }
  int nconmax() const { return m->nconmax; }
  int nuserdata() const { return m->nuserdata; }
  int nsensordata() const { return m->nsensordata; }
  int npluginstate() const { return m->npluginstate; }
  int narena() const { return m->narena; }
  int nbuffer() const { return m->nbuffer; }
  val qpos0() const { return val(typed_memory_view(m->nq * 1, m->qpos0)); }
  val qpos_spring() const { return val(typed_memory_view(m->nq * 1, m->qpos_spring)); }
  val body_parentid() const { return val(typed_memory_view(m->nbody * 1, m->body_parentid)); }
  val body_rootid() const { return val(typed_memory_view(m->nbody * 1, m->body_rootid)); }
  val body_weldid() const { return val(typed_memory_view(m->nbody * 1, m->body_weldid)); }
  val body_mocapid() const { return val(typed_memory_view(m->nbody * 1, m->body_mocapid)); }
  val body_jntnum() const { return val(typed_memory_view(m->nbody * 1, m->body_jntnum)); }
  val body_jntadr() const { return val(typed_memory_view(m->nbody * 1, m->body_jntadr)); }
  val body_dofnum() const { return val(typed_memory_view(m->nbody * 1, m->body_dofnum)); }
  val body_dofadr() const { return val(typed_memory_view(m->nbody * 1, m->body_dofadr)); }
  val body_treeid() const { return val(typed_memory_view(m->nbody * 1, m->body_treeid)); }
  val body_geomnum() const { return val(typed_memory_view(m->nbody * 1, m->body_geomnum)); }
  val body_geomadr() const { return val(typed_memory_view(m->nbody * 1, m->body_geomadr)); }
  val body_simple() const { return val(typed_memory_view(m->nbody * 1, m->body_simple)); }
  val body_sameframe() const { return val(typed_memory_view(m->nbody * 1, m->body_sameframe)); }
  val body_pos() const { return val(typed_memory_view(m->nbody * 3, m->body_pos)); }
  val body_quat() const { return val(typed_memory_view(m->nbody * 4, m->body_quat)); }
  val body_ipos() const { return val(typed_memory_view(m->nbody * 3, m->body_ipos)); }
  val body_iquat() const { return val(typed_memory_view(m->nbody * 4, m->body_iquat)); }
  val body_mass() const { return val(typed_memory_view(m->nbody * 1, m->body_mass)); }
  val body_subtreemass() const { return val(typed_memory_view(m->nbody * 1, m->body_subtreemass)); }
  val body_inertia() const { return val(typed_memory_view(m->nbody * 3, m->body_inertia)); }
  val body_invweight0() const { return val(typed_memory_view(m->nbody * 2, m->body_invweight0)); }
  val body_gravcomp() const { return val(typed_memory_view(m->nbody * 1, m->body_gravcomp)); }
  val body_margin() const { return val(typed_memory_view(m->nbody * 1, m->body_margin)); }
  val body_user() const { return val(typed_memory_view(m->nbody * m->nuser_body, m->body_user)); }
  val body_plugin() const { return val(typed_memory_view(m->nbody * 1, m->body_plugin)); }
  val body_contype() const { return val(typed_memory_view(m->nbody * 1, m->body_contype)); }
  val body_conaffinity() const { return val(typed_memory_view(m->nbody * 1, m->body_conaffinity)); }
  val body_bvhadr() const { return val(typed_memory_view(m->nbody * 1, m->body_bvhadr)); }
  val body_bvhnum() const { return val(typed_memory_view(m->nbody * 1, m->body_bvhnum)); }
  val bvh_depth() const { return val(typed_memory_view(m->nbvh * 1, m->bvh_depth)); }
  val bvh_child() const { return val(typed_memory_view(m->nbvh * 2, m->bvh_child)); }
  val bvh_nodeid() const { return val(typed_memory_view(m->nbvh * 1, m->bvh_nodeid)); }
  val bvh_aabb() const { return val(typed_memory_view(m->nbvhstatic * 6, m->bvh_aabb)); }
  val jnt_type() const { return val(typed_memory_view(m->njnt * 1, m->jnt_type)); }
  val jnt_qposadr() const { return val(typed_memory_view(m->njnt * 1, m->jnt_qposadr)); }
  val jnt_dofadr() const { return val(typed_memory_view(m->njnt * 1, m->jnt_dofadr)); }
  val jnt_bodyid() const { return val(typed_memory_view(m->njnt * 1, m->jnt_bodyid)); }
  val jnt_group() const { return val(typed_memory_view(m->njnt * 1, m->jnt_group)); }
  val jnt_limited() const { return val(typed_memory_view(m->njnt * 1, m->jnt_limited)); }
  val jnt_actfrclimited() const { return val(typed_memory_view(m->njnt * 1, m->jnt_actfrclimited)); }
  val jnt_actgravcomp() const { return val(typed_memory_view(m->njnt * 1, m->jnt_actgravcomp)); }
  val jnt_solref() const { return val(typed_memory_view(m->njnt * mjNREF, m->jnt_solref)); }
  val jnt_solimp() const { return val(typed_memory_view(m->njnt * mjNIMP, m->jnt_solimp)); }
  val jnt_pos() const { return val(typed_memory_view(m->njnt * 3, m->jnt_pos)); }
  val jnt_axis() const { return val(typed_memory_view(m->njnt * 3, m->jnt_axis)); }
  val jnt_stiffness() const { return val(typed_memory_view(m->njnt * 1, m->jnt_stiffness)); }
  val jnt_range() const { return val(typed_memory_view(m->njnt * 2, m->jnt_range)); }
  val jnt_actfrcrange() const { return val(typed_memory_view(m->njnt * 2, m->jnt_actfrcrange)); }
  val jnt_margin() const { return val(typed_memory_view(m->njnt * 1, m->jnt_margin)); }
  val jnt_user() const { return val(typed_memory_view(m->njnt * m->nuser_jnt, m->jnt_user)); }
  val dof_bodyid() const { return val(typed_memory_view(m->nv * 1, m->dof_bodyid)); }
  val dof_jntid() const { return val(typed_memory_view(m->nv * 1, m->dof_jntid)); }
  val dof_parentid() const { return val(typed_memory_view(m->nv * 1, m->dof_parentid)); }
  val dof_treeid() const { return val(typed_memory_view(m->nv * 1, m->dof_treeid)); }
  val dof_Madr() const { return val(typed_memory_view(m->nv * 1, m->dof_Madr)); }
  val dof_simplenum() const { return val(typed_memory_view(m->nv * 1, m->dof_simplenum)); }
  val dof_solref() const { return val(typed_memory_view(m->nv * mjNREF, m->dof_solref)); }
  val dof_solimp() const { return val(typed_memory_view(m->nv * mjNIMP, m->dof_solimp)); }
  val dof_frictionloss() const { return val(typed_memory_view(m->nv * 1, m->dof_frictionloss)); }
  val dof_armature() const { return val(typed_memory_view(m->nv * 1, m->dof_armature)); }
  val dof_damping() const { return val(typed_memory_view(m->nv * 1, m->dof_damping)); }
  val dof_invweight0() const { return val(typed_memory_view(m->nv * 1, m->dof_invweight0)); }
  val dof_M0() const { return val(typed_memory_view(m->nv * 1, m->dof_M0)); }
  val geom_type() const { return val(typed_memory_view(m->ngeom * 1, m->geom_type)); }
  val geom_contype() const { return val(typed_memory_view(m->ngeom * 1, m->geom_contype)); }
  val geom_conaffinity() const { return val(typed_memory_view(m->ngeom * 1, m->geom_conaffinity)); }
  val geom_condim() const { return val(typed_memory_view(m->ngeom * 1, m->geom_condim)); }
  val geom_bodyid() const { return val(typed_memory_view(m->ngeom * 1, m->geom_bodyid)); }
  val geom_dataid() const { return val(typed_memory_view(m->ngeom * 1, m->geom_dataid)); }
  val geom_matid() const { return val(typed_memory_view(m->ngeom * 1, m->geom_matid)); }
  val geom_group() const { return val(typed_memory_view(m->ngeom * 1, m->geom_group)); }
  val geom_priority() const { return val(typed_memory_view(m->ngeom * 1, m->geom_priority)); }
  val geom_plugin() const { return val(typed_memory_view(m->ngeom * 1, m->geom_plugin)); }
  val geom_sameframe() const { return val(typed_memory_view(m->ngeom * 1, m->geom_sameframe)); }
  val geom_solmix() const { return val(typed_memory_view(m->ngeom * 1, m->geom_solmix)); }
  val geom_solref() const { return val(typed_memory_view(m->ngeom * mjNREF, m->geom_solref)); }
  val geom_solimp() const { return val(typed_memory_view(m->ngeom * mjNIMP, m->geom_solimp)); }
  val geom_size() const { return val(typed_memory_view(m->ngeom * 3, m->geom_size)); }
  val geom_aabb() const { return val(typed_memory_view(m->ngeom * 6, m->geom_aabb)); }
  val geom_rbound() const { return val(typed_memory_view(m->ngeom * 1, m->geom_rbound)); }
  val geom_pos() const { return val(typed_memory_view(m->ngeom * 3, m->geom_pos)); }
  val geom_quat() const { return val(typed_memory_view(m->ngeom * 4, m->geom_quat)); }
  val geom_friction() const { return val(typed_memory_view(m->ngeom * 3, m->geom_friction)); }
  val geom_margin() const { return val(typed_memory_view(m->ngeom * 1, m->geom_margin)); }
  val geom_gap() const { return val(typed_memory_view(m->ngeom * 1, m->geom_gap)); }
  val geom_fluid() const { return val(typed_memory_view(m->ngeom * mjNFLUID, m->geom_fluid)); }
  val geom_user() const { return val(typed_memory_view(m->ngeom * m->nuser_geom, m->geom_user)); }
  val geom_rgba() const { return val(typed_memory_view(m->ngeom * 4, m->geom_rgba)); }
  val site_type() const { return val(typed_memory_view(m->nsite * 1, m->site_type)); }
  val site_bodyid() const { return val(typed_memory_view(m->nsite * 1, m->site_bodyid)); }
  val site_matid() const { return val(typed_memory_view(m->nsite * 1, m->site_matid)); }
  val site_group() const { return val(typed_memory_view(m->nsite * 1, m->site_group)); }
  val site_sameframe() const { return val(typed_memory_view(m->nsite * 1, m->site_sameframe)); }
  val site_size() const { return val(typed_memory_view(m->nsite * 3, m->site_size)); }
  val site_pos() const { return val(typed_memory_view(m->nsite * 3, m->site_pos)); }
  val site_quat() const { return val(typed_memory_view(m->nsite * 4, m->site_quat)); }
  val site_user() const { return val(typed_memory_view(m->nsite * m->nuser_site, m->site_user)); }
  val site_rgba() const { return val(typed_memory_view(m->nsite * 4, m->site_rgba)); }
  val cam_mode() const { return val(typed_memory_view(m->ncam * 1, m->cam_mode)); }
  val cam_bodyid() const { return val(typed_memory_view(m->ncam * 1, m->cam_bodyid)); }
  val cam_targetbodyid() const { return val(typed_memory_view(m->ncam * 1, m->cam_targetbodyid)); }
  val cam_pos() const { return val(typed_memory_view(m->ncam * 3, m->cam_pos)); }
  val cam_quat() const { return val(typed_memory_view(m->ncam * 4, m->cam_quat)); }
  val cam_poscom0() const { return val(typed_memory_view(m->ncam * 3, m->cam_poscom0)); }
  val cam_pos0() const { return val(typed_memory_view(m->ncam * 3, m->cam_pos0)); }
  val cam_mat0() const { return val(typed_memory_view(m->ncam * 9, m->cam_mat0)); }
  val cam_orthographic() const { return val(typed_memory_view(m->ncam * 1, m->cam_orthographic)); }
  val cam_fovy() const { return val(typed_memory_view(m->ncam * 1, m->cam_fovy)); }
  val cam_ipd() const { return val(typed_memory_view(m->ncam * 1, m->cam_ipd)); }
  val cam_resolution() const { return val(typed_memory_view(m->ncam * 2, m->cam_resolution)); }
  val cam_sensorsize() const { return val(typed_memory_view(m->ncam * 2, m->cam_sensorsize)); }
  val cam_intrinsic() const { return val(typed_memory_view(m->ncam * 4, m->cam_intrinsic)); }
  val cam_user() const { return val(typed_memory_view(m->ncam * m->nuser_cam, m->cam_user)); }
  val light_mode() const { return val(typed_memory_view(m->nlight * 1, m->light_mode)); }
  val light_bodyid() const { return val(typed_memory_view(m->nlight * 1, m->light_bodyid)); }
  val light_targetbodyid() const { return val(typed_memory_view(m->nlight * 1, m->light_targetbodyid)); }
  val light_directional() const { return val(typed_memory_view(m->nlight * 1, m->light_directional)); }
  val light_castshadow() const { return val(typed_memory_view(m->nlight * 1, m->light_castshadow)); }
  val light_bulbradius() const { return val(typed_memory_view(m->nlight * 1, m->light_bulbradius)); }
  val light_active() const { return val(typed_memory_view(m->nlight * 1, m->light_active)); }
  val light_pos() const { return val(typed_memory_view(m->nlight * 3, m->light_pos)); }
  val light_dir() const { return val(typed_memory_view(m->nlight * 3, m->light_dir)); }
  val light_poscom0() const { return val(typed_memory_view(m->nlight * 3, m->light_poscom0)); }
  val light_pos0() const { return val(typed_memory_view(m->nlight * 3, m->light_pos0)); }
  val light_dir0() const { return val(typed_memory_view(m->nlight * 3, m->light_dir0)); }
  val light_attenuation() const { return val(typed_memory_view(m->nlight * 3, m->light_attenuation)); }
  val light_cutoff() const { return val(typed_memory_view(m->nlight * 1, m->light_cutoff)); }
  val light_exponent() const { return val(typed_memory_view(m->nlight * 1, m->light_exponent)); }
  val light_ambient() const { return val(typed_memory_view(m->nlight * 3, m->light_ambient)); }
  val light_diffuse() const { return val(typed_memory_view(m->nlight * 3, m->light_diffuse)); }
  val light_specular() const { return val(typed_memory_view(m->nlight * 3, m->light_specular)); }
  val flex_contype() const { return val(typed_memory_view(m->nflex * 1, m->flex_contype)); }
  val flex_conaffinity() const { return val(typed_memory_view(m->nflex * 1, m->flex_conaffinity)); }
  val flex_condim() const { return val(typed_memory_view(m->nflex * 1, m->flex_condim)); }
  val flex_priority() const { return val(typed_memory_view(m->nflex * 1, m->flex_priority)); }
  val flex_solmix() const { return val(typed_memory_view(m->nflex * 1, m->flex_solmix)); }
  val flex_solref() const { return val(typed_memory_view(m->nflex * mjNREF, m->flex_solref)); }
  val flex_solimp() const { return val(typed_memory_view(m->nflex * mjNIMP, m->flex_solimp)); }
  val flex_friction() const { return val(typed_memory_view(m->nflex * 3, m->flex_friction)); }
  val flex_margin() const { return val(typed_memory_view(m->nflex * 1, m->flex_margin)); }
  val flex_gap() const { return val(typed_memory_view(m->nflex * 1, m->flex_gap)); }
  val flex_internal() const { return val(typed_memory_view(m->nflex * 1, m->flex_internal)); }
  val flex_selfcollide() const { return val(typed_memory_view(m->nflex * 1, m->flex_selfcollide)); }
  val flex_activelayers() const { return val(typed_memory_view(m->nflex * 1, m->flex_activelayers)); }
  val flex_dim() const { return val(typed_memory_view(m->nflex * 1, m->flex_dim)); }
  val flex_matid() const { return val(typed_memory_view(m->nflex * 1, m->flex_matid)); }
  val flex_group() const { return val(typed_memory_view(m->nflex * 1, m->flex_group)); }
  val flex_vertadr() const { return val(typed_memory_view(m->nflex * 1, m->flex_vertadr)); }
  val flex_vertnum() const { return val(typed_memory_view(m->nflex * 1, m->flex_vertnum)); }
  val flex_edgeadr() const { return val(typed_memory_view(m->nflex * 1, m->flex_edgeadr)); }
  val flex_edgenum() const { return val(typed_memory_view(m->nflex * 1, m->flex_edgenum)); }
  val flex_elemadr() const { return val(typed_memory_view(m->nflex * 1, m->flex_elemadr)); }
  val flex_elemnum() const { return val(typed_memory_view(m->nflex * 1, m->flex_elemnum)); }
  val flex_elemdataadr() const { return val(typed_memory_view(m->nflex * 1, m->flex_elemdataadr)); }
  val flex_elemedgeadr() const { return val(typed_memory_view(m->nflex * 1, m->flex_elemedgeadr)); }
  val flex_shellnum() const { return val(typed_memory_view(m->nflex * 1, m->flex_shellnum)); }
  val flex_shelldataadr() const { return val(typed_memory_view(m->nflex * 1, m->flex_shelldataadr)); }
  val flex_evpairadr() const { return val(typed_memory_view(m->nflex * 1, m->flex_evpairadr)); }
  val flex_evpairnum() const { return val(typed_memory_view(m->nflex * 1, m->flex_evpairnum)); }
  val flex_texcoordadr() const { return val(typed_memory_view(m->nflex * 1, m->flex_texcoordadr)); }
  val flex_vertbodyid() const { return val(typed_memory_view(m->nflexvert * 1, m->flex_vertbodyid)); }
  val flex_edge() const { return val(typed_memory_view(m->nflexedge * 2, m->flex_edge)); }
  val flex_elem() const { return val(typed_memory_view(m->nflexelemdata * 1, m->flex_elem)); }
  val flex_elemedge() const { return val(typed_memory_view(m->nflexelemedge * 1, m->flex_elemedge)); }
  val flex_elemlayer() const { return val(typed_memory_view(m->nflexelem * 1, m->flex_elemlayer)); }
  val flex_shell() const { return val(typed_memory_view(m->nflexshelldata * 1, m->flex_shell)); }
  val flex_evpair() const { return val(typed_memory_view(m->nflexevpair * 2, m->flex_evpair)); }
  val flex_vert() const { return val(typed_memory_view(m->nflexvert * 3, m->flex_vert)); }
  val flex_xvert0() const { return val(typed_memory_view(m->nflexvert * 3, m->flex_vert0)); }
  val flexedge_length0() const { return val(typed_memory_view(m->nflexedge * 1, m->flexedge_length0)); }
  val flexedge_invweight0() const { return val(typed_memory_view(m->nflexedge * 1, m->flexedge_invweight0)); }
  val flex_radius() const { return val(typed_memory_view(m->nflex * 1, m->flex_radius)); }
  val flex_stiffness() const { return val(typed_memory_view(m->nflexelem * 21, m->flex_stiffness)); }
  val flex_damping() const { return val(typed_memory_view(m->nflex * 1, m->flex_damping)); }
  val flex_edgestiffness() const { return val(typed_memory_view(m->nflex * 1, m->flex_edgestiffness)); }
  val flex_edgedamping() const { return val(typed_memory_view(m->nflex * 1, m->flex_edgedamping)); }
  val flex_edgeequality() const { return val(typed_memory_view(m->nflex * 1, m->flex_edgeequality)); }
  val flex_rigid() const { return val(typed_memory_view(m->nflex * 1, m->flex_rigid)); }
  val flexedge_rigid() const { return val(typed_memory_view(m->nflexedge * 1, m->flexedge_rigid)); }
  val flex_centered() const { return val(typed_memory_view(m->nflex * 1, m->flex_centered)); }
  val flex_flatskin() const { return val(typed_memory_view(m->nflex * 1, m->flex_flatskin)); }
  val flex_bvhadr() const { return val(typed_memory_view(m->nflex * 1, m->flex_bvhadr)); }
  val flex_bvhnum() const { return val(typed_memory_view(m->nflex * 1, m->flex_bvhnum)); }
  val flex_rgba() const { return val(typed_memory_view(m->nflex * 4, m->flex_rgba)); }
  val flex_texcoord() const { return val(typed_memory_view(m->nflextexcoord * 2, m->flex_texcoord)); }
  val mesh_vertadr() const { return val(typed_memory_view(m->nmesh * 1, m->mesh_vertadr)); }
  val mesh_vertnum() const { return val(typed_memory_view(m->nmesh * 1, m->mesh_vertnum)); }
  val mesh_faceadr() const { return val(typed_memory_view(m->nmesh * 1, m->mesh_faceadr)); }
  val mesh_facenum() const { return val(typed_memory_view(m->nmesh * 1, m->mesh_facenum)); }
  val mesh_bvhadr() const { return val(typed_memory_view(m->nmesh * 1, m->mesh_bvhadr)); }
  val mesh_bvhnum() const { return val(typed_memory_view(m->nmesh * 1, m->mesh_bvhnum)); }
  val mesh_normaladr() const { return val(typed_memory_view(m->nmesh * 1, m->mesh_normaladr)); }
  val mesh_normalnum() const { return val(typed_memory_view(m->nmesh * 1, m->mesh_normalnum)); }
  val mesh_texcoordadr() const { return val(typed_memory_view(m->nmesh * 1, m->mesh_texcoordadr)); }
  val mesh_texcoordnum() const { return val(typed_memory_view(m->nmesh * 1, m->mesh_texcoordnum)); }
  val mesh_graphadr() const { return val(typed_memory_view(m->nmesh * 1, m->mesh_graphadr)); }
  val mesh_vert() const { return val(typed_memory_view(m->nmeshvert * 3, m->mesh_vert)); }
  val mesh_normal() const { return val(typed_memory_view(m->nmeshnormal * 3, m->mesh_normal)); }
  val mesh_texcoord() const { return val(typed_memory_view(m->nmeshtexcoord * 2, m->mesh_texcoord)); }
  val mesh_face() const { return val(typed_memory_view(m->nmeshface * 3, m->mesh_face)); }
  val mesh_facenormal() const { return val(typed_memory_view(m->nmeshface * 3, m->mesh_facenormal)); }
  val mesh_facetexcoord() const { return val(typed_memory_view(m->nmeshface * 3, m->mesh_facetexcoord)); }
  val mesh_graph() const { return val(typed_memory_view(m->nmeshgraph * 1, m->mesh_graph)); }
  val mesh_scale() const { return val(typed_memory_view(m->nmesh * 3, m->mesh_scale)); }
  val mesh_pos() const { return val(typed_memory_view(m->nmesh * 3, m->mesh_pos)); }
  val mesh_quat() const { return val(typed_memory_view(m->nmesh * 4, m->mesh_quat)); }
  val mesh_pathadr() const { return val(typed_memory_view(m->nmesh * 1, m->mesh_pathadr)); }
  val skin_matid() const { return val(typed_memory_view(m->nskin * 1, m->skin_matid)); }
  val skin_group() const { return val(typed_memory_view(m->nskin * 1, m->skin_group)); }
  val skin_rgba() const { return val(typed_memory_view(m->nskin * 4, m->skin_rgba)); }
  val skin_inflate() const { return val(typed_memory_view(m->nskin * 1, m->skin_inflate)); }
  val skin_vertadr() const { return val(typed_memory_view(m->nskin * 1, m->skin_vertadr)); }
  val skin_vertnum() const { return val(typed_memory_view(m->nskin * 1, m->skin_vertnum)); }
  val skin_texcoordadr() const { return val(typed_memory_view(m->nskin * 1, m->skin_texcoordadr)); }
  val skin_faceadr() const { return val(typed_memory_view(m->nskin * 1, m->skin_faceadr)); }
  val skin_facenum() const { return val(typed_memory_view(m->nskin * 1, m->skin_facenum)); }
  val skin_boneadr() const { return val(typed_memory_view(m->nskin * 1, m->skin_boneadr)); }
  val skin_bonenum() const { return val(typed_memory_view(m->nskin * 1, m->skin_bonenum)); }
  val skin_vert() const { return val(typed_memory_view(m->nskinvert * 3, m->skin_vert)); }
  val skin_texcoord() const { return val(typed_memory_view(m->nskintexvert * 2, m->skin_texcoord)); }
  val skin_face() const { return val(typed_memory_view(m->nskinface * 3, m->skin_face)); }
  val skin_bonevertadr() const { return val(typed_memory_view(m->nskinbone * 1, m->skin_bonevertadr)); }
  val skin_bonevertnum() const { return val(typed_memory_view(m->nskinbone * 1, m->skin_bonevertnum)); }
  val skin_bonebindpos() const { return val(typed_memory_view(m->nskinbone * 3, m->skin_bonebindpos)); }
  val skin_bonebindquat() const { return val(typed_memory_view(m->nskinbone * 4, m->skin_bonebindquat)); }
  val skin_bonebodyid() const { return val(typed_memory_view(m->nskinbone * 1, m->skin_bonebodyid)); }
  val skin_bonevertid() const { return val(typed_memory_view(m->nskinbonevert * 1, m->skin_bonevertid)); }
  val skin_bonevertweight() const { return val(typed_memory_view(m->nskinbonevert * 1, m->skin_bonevertweight)); }
  val skin_pathadr() const { return val(typed_memory_view(m->nskin * 1, m->skin_pathadr)); }
  val hfield_size() const { return val(typed_memory_view(m->nhfield * 4, m->hfield_size)); }
  val hfield_nrow() const { return val(typed_memory_view(m->nhfield * 1, m->hfield_nrow)); }
  val hfield_ncol() const { return val(typed_memory_view(m->nhfield * 1, m->hfield_ncol)); }
  val hfield_adr() const { return val(typed_memory_view(m->nhfield * 1, m->hfield_adr)); }
  val hfield_data() const { return val(typed_memory_view(m->nhfielddata * 1, m->hfield_data)); }
  val hfield_pathadr() const { return val(typed_memory_view(m->nhfield * 1, m->hfield_pathadr)); }
  val tex_type() const { return val(typed_memory_view(m->ntex * 1, m->tex_type)); }
  val tex_height() const { return val(typed_memory_view(m->ntex * 1, m->tex_height)); }
  val tex_width() const { return val(typed_memory_view(m->ntex * 1, m->tex_width)); }
  val tex_nchannel() const { return val(typed_memory_view(m->ntex * 1, m->tex_nchannel)); }
  val tex_adr() const { return val(typed_memory_view(m->ntex * 1, m->tex_adr)); }
  val tex_data() const { return val(typed_memory_view(m->ntexdata * 1, m->tex_data)); }
  val tex_pathadr() const { return val(typed_memory_view(m->ntex * 1, m->tex_pathadr)); }
  val mat_texid() const { return val(typed_memory_view(m->nmat * mjNTEXROLE, m->mat_texid)); }
  val mat_texuniform() const { return val(typed_memory_view(m->nmat * 1, m->mat_texuniform)); }
  val mat_texrepeat() const { return val(typed_memory_view(m->nmat * 2, m->mat_texrepeat)); }
  val mat_emission() const { return val(typed_memory_view(m->nmat * 1, m->mat_emission)); }
  val mat_specular() const { return val(typed_memory_view(m->nmat * 1, m->mat_specular)); }
  val mat_shininess() const { return val(typed_memory_view(m->nmat * 1, m->mat_shininess)); }
  val mat_reflectance() const { return val(typed_memory_view(m->nmat * 1, m->mat_reflectance)); }
  val mat_metallic() const { return val(typed_memory_view(m->nmat * 1, m->mat_metallic)); }
  val mat_roughness() const { return val(typed_memory_view(m->nmat * 1, m->mat_roughness)); }
  val mat_rgba() const { return val(typed_memory_view(m->nmat * 4, m->mat_rgba)); }
  val pair_dim() const { return val(typed_memory_view(m->npair * 1, m->pair_dim)); }
  val pair_geom1() const { return val(typed_memory_view(m->npair * 1, m->pair_geom1)); }
  val pair_geom2() const { return val(typed_memory_view(m->npair * 1, m->pair_geom2)); }
  val pair_signature() const { return val(typed_memory_view(m->npair * 1, m->pair_signature)); }
  val pair_solref() const { return val(typed_memory_view(m->npair * mjNREF, m->pair_solref)); }
  val pair_solreffriction() const { return val(typed_memory_view(m->npair * mjNREF, m->pair_solreffriction)); }
  val pair_solimp() const { return val(typed_memory_view(m->npair * mjNIMP, m->pair_solimp)); }
  val pair_margin() const { return val(typed_memory_view(m->npair * 1, m->pair_margin)); }
  val pair_gap() const { return val(typed_memory_view(m->npair * 1, m->pair_gap)); }
  val pair_friction() const { return val(typed_memory_view(m->npair * 5, m->pair_friction)); }
  val exclude_signature() const { return val(typed_memory_view(m->nexclude * 1, m->exclude_signature)); }
  val eq_type() const { return val(typed_memory_view(m->neq * 1, m->eq_type)); }
  val eq_obj1id() const { return val(typed_memory_view(m->neq * 1, m->eq_obj1id)); }
  val eq_obj2id() const { return val(typed_memory_view(m->neq * 1, m->eq_obj2id)); }
  val eq_objtype() const { return val(typed_memory_view(m->neq * 1, m->eq_objtype)); }
  val eq_active0() const { return val(typed_memory_view(m->neq * 1, m->eq_active0)); }
  val eq_solref() const { return val(typed_memory_view(m->neq * mjNREF, m->eq_solref)); }
  val eq_solimp() const { return val(typed_memory_view(m->neq * mjNIMP, m->eq_solimp)); }
  val eq_data() const { return val(typed_memory_view(m->neq * mjNEQDATA, m->eq_data)); }
  val tendon_adr() const { return val(typed_memory_view(m->ntendon * 1, m->tendon_adr)); }
  val tendon_num() const { return val(typed_memory_view(m->ntendon * 1, m->tendon_num)); }
  val tendon_matid() const { return val(typed_memory_view(m->ntendon * 1, m->tendon_matid)); }
  val tendon_group() const { return val(typed_memory_view(m->ntendon * 1, m->tendon_group)); }
  val tendon_limited() const { return val(typed_memory_view(m->ntendon * 1, m->tendon_limited)); }
  val tendon_width() const { return val(typed_memory_view(m->ntendon * 1, m->tendon_width)); }
  val tendon_solref_lim() const { return val(typed_memory_view(m->ntendon * mjNREF, m->tendon_solref_lim)); }
  val tendon_solimp_lim() const { return val(typed_memory_view(m->ntendon * mjNIMP, m->tendon_solimp_lim)); }
  val tendon_solref_fri() const { return val(typed_memory_view(m->ntendon * mjNREF, m->tendon_solref_fri)); }
  val tendon_solimp_fri() const { return val(typed_memory_view(m->ntendon * mjNIMP, m->tendon_solimp_fri)); }
  val tendon_range() const { return val(typed_memory_view(m->ntendon * 2, m->tendon_range)); }
  val tendon_margin() const { return val(typed_memory_view(m->ntendon * 1, m->tendon_margin)); }
  val tendon_stiffness() const { return val(typed_memory_view(m->ntendon * 1, m->tendon_stiffness)); }
  val tendon_damping() const { return val(typed_memory_view(m->ntendon * 1, m->tendon_damping)); }
  val tendon_frictionloss() const { return val(typed_memory_view(m->ntendon * 1, m->tendon_frictionloss)); }
  val tendon_lengthspring() const { return val(typed_memory_view(m->ntendon * 2, m->tendon_lengthspring)); }
  val tendon_length0() const { return val(typed_memory_view(m->ntendon * 1, m->tendon_length0)); }
  val tendon_invweight0() const { return val(typed_memory_view(m->ntendon * 1, m->tendon_invweight0)); }
  val tendon_user() const { return val(typed_memory_view(m->ntendon * m->nuser_tendon, m->tendon_user)); }
  val tendon_rgba() const { return val(typed_memory_view(m->ntendon * 4, m->tendon_rgba)); }
  val wrap_type() const { return val(typed_memory_view(m->nwrap * 1, m->wrap_type)); }
  val wrap_objid() const { return val(typed_memory_view(m->nwrap * 1, m->wrap_objid)); }
  val wrap_prm() const { return val(typed_memory_view(m->nwrap * 1, m->wrap_prm)); }
  val actuator_trntype() const { return val(typed_memory_view(m->nu * 1, m->actuator_trntype)); }
  val actuator_dyntype() const { return val(typed_memory_view(m->nu * 1, m->actuator_dyntype)); }
  val actuator_gaintype() const { return val(typed_memory_view(m->nu * 1, m->actuator_gaintype)); }
  val actuator_biastype() const { return val(typed_memory_view(m->nu * 1, m->actuator_biastype)); }
  val actuator_trnid() const { return val(typed_memory_view(m->nu * 2, m->actuator_trnid)); }
  val actuator_actadr() const { return val(typed_memory_view(m->nu * 1, m->actuator_actadr)); }
  val actuator_actnum() const { return val(typed_memory_view(m->nu * 1, m->actuator_actnum)); }
  val actuator_group() const { return val(typed_memory_view(m->nu * 1, m->actuator_group)); }
  val actuator_ctrllimited() const { return val(typed_memory_view(m->nu * 1, m->actuator_ctrllimited)); }
  val actuator_forcelimited() const { return val(typed_memory_view(m->nu * 1, m->actuator_forcelimited)); }
  val actuator_actlimited() const { return val(typed_memory_view(m->nu * 1, m->actuator_actlimited)); }
  val actuator_dynprm() const { return val(typed_memory_view(m->nu * mjNDYN, m->actuator_dynprm)); }
  val actuator_gainprm() const { return val(typed_memory_view(m->nu * mjNGAIN, m->actuator_gainprm)); }
  val actuator_biasprm() const { return val(typed_memory_view(m->nu * mjNBIAS, m->actuator_biasprm)); }
  val actuator_actearly() const { return val(typed_memory_view(m->nu * 1, m->actuator_actearly)); }
  val actuator_ctrlrange() const { return val(typed_memory_view(m->nu * 2, m->actuator_ctrlrange)); }
  val actuator_forcerange() const { return val(typed_memory_view(m->nu * 2, m->actuator_forcerange)); }
  val actuator_actrange() const { return val(typed_memory_view(m->nu * 2, m->actuator_actrange)); }
  val actuator_gear() const { return val(typed_memory_view(m->nu * 6, m->actuator_gear)); }
  val actuator_cranklength() const { return val(typed_memory_view(m->nu * 1, m->actuator_cranklength)); }
  val actuator_acc0() const { return val(typed_memory_view(m->nu * 1, m->actuator_acc0)); }
  val actuator_length0() const { return val(typed_memory_view(m->nu * 1, m->actuator_length0)); }
  val actuator_lengthrange() const { return val(typed_memory_view(m->nu * 2, m->actuator_lengthrange)); }
  val actuator_user() const { return val(typed_memory_view(m->nu * m->nuser_actuator, m->actuator_user)); }
  val actuator_plugin() const { return val(typed_memory_view(m->nu * 1, m->actuator_plugin)); }
  val sensor_type() const { return val(typed_memory_view(m->nsensor * 1, m->sensor_type)); }
  val sensor_datatype() const { return val(typed_memory_view(m->nsensor * 1, m->sensor_datatype)); }
  val sensor_needstage() const { return val(typed_memory_view(m->nsensor * 1, m->sensor_needstage)); }
  val sensor_objtype() const { return val(typed_memory_view(m->nsensor * 1, m->sensor_objtype)); }
  val sensor_objid() const { return val(typed_memory_view(m->nsensor * 1, m->sensor_objid)); }
  val sensor_reftype() const { return val(typed_memory_view(m->nsensor * 1, m->sensor_reftype)); }
  val sensor_refid() const { return val(typed_memory_view(m->nsensor * 1, m->sensor_refid)); }
  val sensor_dim() const { return val(typed_memory_view(m->nsensor * 1, m->sensor_dim)); }
  val sensor_adr() const { return val(typed_memory_view(m->nsensor * 1, m->sensor_adr)); }
  val sensor_cutoff() const { return val(typed_memory_view(m->nsensor * 1, m->sensor_cutoff)); }
  val sensor_noise() const { return val(typed_memory_view(m->nsensor * 1, m->sensor_noise)); }
  val sensor_user() const { return val(typed_memory_view(m->nsensor * m->nuser_sensor, m->sensor_user)); }
  val sensor_plugin() const { return val(typed_memory_view(m->nsensor * 1, m->sensor_plugin)); }
  val plugin() const { return val(typed_memory_view(m->nplugin * 1, m->plugin)); }
  val plugin_stateadr() const { return val(typed_memory_view(m->nplugin * 1, m->plugin_stateadr)); }
  val plugin_statenum() const { return val(typed_memory_view(m->nplugin * 1, m->plugin_statenum)); }
  val plugin_attr() const { return val(typed_memory_view(m->npluginattr * 1, m->plugin_attr)); }
  val plugin_attradr() const { return val(typed_memory_view(m->nplugin * 1, m->plugin_attradr)); }
  val numeric_adr() const { return val(typed_memory_view(m->nnumeric * 1, m->numeric_adr)); }
  val numeric_size() const { return val(typed_memory_view(m->nnumeric * 1, m->numeric_size)); }
  val numeric_data() const { return val(typed_memory_view(m->nnumericdata * 1, m->numeric_data)); }
  val text_adr() const { return val(typed_memory_view(m->ntext * 1, m->text_adr)); }
  val text_size() const { return val(typed_memory_view(m->ntext * 1, m->text_size)); }
  val text_data() const { return val(typed_memory_view(m->ntextdata * 1, m->text_data)); }
  val tuple_adr() const { return val(typed_memory_view(m->ntuple * 1, m->tuple_adr)); }
  val tuple_size() const { return val(typed_memory_view(m->ntuple * 1, m->tuple_size)); }
  val tuple_objtype() const { return val(typed_memory_view(m->ntupledata * 1, m->tuple_objtype)); }
  val tuple_objid() const { return val(typed_memory_view(m->ntupledata * 1, m->tuple_objid)); }
  val tuple_objprm() const { return val(typed_memory_view(m->ntupledata * 1, m->tuple_objprm)); }
  val key_time() const { return val(typed_memory_view(m->nkey * 1, m->key_time)); }
  val key_qpos() const { return val(typed_memory_view(m->nkey * m->nq, m->key_qpos)); }
  val key_qvel() const { return val(typed_memory_view(m->nkey * m->nv, m->key_qvel)); }
  val key_act() const { return val(typed_memory_view(m->nkey * m->na, m->key_act)); }
  val key_mpos() const { return val(typed_memory_view(m->nkey * 3*m->nmocap, m->key_mpos)); }
  val key_mquat() const { return val(typed_memory_view(m->nkey * 4* m->nmocap, m->key_mquat)); }
  val key_ctrl() const { return val(typed_memory_view(m->nkey * m->nu, m->key_ctrl)); }
  val name_bodyadr() const { return val(typed_memory_view(m->nbody * 1, m->name_bodyadr)); }
  val name_jntadr() const { return val(typed_memory_view(m->njnt * 1, m->name_jntadr)); }
  val name_geomadr() const { return val(typed_memory_view(m->ngeom * 1, m->name_geomadr)); }
  val name_siteadr() const { return val(typed_memory_view(m->nsite * 1, m->name_siteadr)); }
  val name_camadr() const { return val(typed_memory_view(m->ncam * 1, m->name_camadr)); }
  val name_lightadr() const { return val(typed_memory_view(m->nlight * 1, m->name_lightadr)); }
  val name_flexadr() const { return val(typed_memory_view(m->nflex * 1, m->name_flexadr)); }
  val name_meshadr() const { return val(typed_memory_view(m->nmesh * 1, m->name_meshadr)); }
  val name_skinadr() const { return val(typed_memory_view(m->nskin * 1, m->name_skinadr)); }
  val name_hfieldadr() const { return val(typed_memory_view(m->nhfield * 1, m->name_hfieldadr)); }
  val name_texadr() const { return val(typed_memory_view(m->ntex * 1, m->name_texadr)); }
  val name_matadr() const { return val(typed_memory_view(m->nmat * 1, m->name_matadr)); }
  val name_pairadr() const { return val(typed_memory_view(m->npair * 1, m->name_pairadr)); }
  val name_excludeadr() const { return val(typed_memory_view(m->nexclude * 1, m->name_excludeadr)); }
  val name_eqadr() const { return val(typed_memory_view(m->neq * 1, m->name_eqadr)); }
  val name_tendonadr() const { return val(typed_memory_view(m->ntendon * 1, m->name_tendonadr)); }
  val name_actuatoradr() const { return val(typed_memory_view(m->nu * 1, m->name_actuatoradr)); }
  val name_sensoradr() const { return val(typed_memory_view(m->nsensor * 1, m->name_sensoradr)); }
  val name_numericadr() const { return val(typed_memory_view(m->nnumeric * 1, m->name_numericadr)); }
  val name_textadr() const { return val(typed_memory_view(m->ntext * 1, m->name_textadr)); }
  val name_tupleadr() const { return val(typed_memory_view(m->ntuple * 1, m->name_tupleadr)); }
  val name_keyadr() const { return val(typed_memory_view(m->nkey * 1, m->name_keyadr)); }
  val name_pluginadr() const { return val(typed_memory_view(m->nplugin * 1, m->name_pluginadr)); }
  val names() const { return val(typed_memory_view(m->nnames * 1, m->names)); }
  val names_map() const { return val(typed_memory_view(m->nnames_map * 1, m->names_map)); }
  val paths() const { return val(typed_memory_view(m->npaths * 1, m->paths)); }

  // clang-format on

private:
  mjModel *m;
  std::string error;
};