
#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "mujoco/mujoco.h"

#include <emscripten/bind.h>
#include <emscripten/fetch.h>
#include <emscripten/val.h>
#include <vector>

using namespace emscripten;

int finish(const char *msg = NULL, mjModel *m = NULL) {
  if (m) {
    mj_deleteModel(m);
  }
  if (msg) {
    std::printf("%s\n", msg);
  }
  return 0;
}

class Model {
public:
  Model() { m = NULL; }
  Model(const std::string filename) {
    if (0 == filename.compare(filename.length() - 3, 3, "mjb")) {
      char error[1000] = "Could not load mjb model";
      m = mj_loadModel(filename.c_str(), 0);
      if (!m) {
        finish(error, m);
      }
    } else {
      char error[1000] = "Could not load xml model";
      m = mj_loadXML(filename.c_str(), 0, error, 1000);
      if (!m) {
        finish(error, m);
      }
    }
  }

  static Model load_from_xml(const std::string filename) {
    return Model(filename);
  }
  static Model load_from_mjb(const std::string filename) {
    return Model(filename);
  }

  mjModel *ptr() { return m; }
  mjModel getVal() { return *m; }
  mjOption getOptions() { return (*m).opt; }
  void free() { return mju_free(m); }

  // MJMODEL_DEFINITIONS
  int nq() const { return m->nq; }
  int nv() const { return m->nv; }
  int nu() const { return m->nu; }
  int na() const { return m->na; }
  int nbody() const { return m->nbody; }
  int njnt() const { return m->njnt; }
  int ngeom() const { return m->ngeom; }
  int nsite() const { return m->nsite; }
  int ncam() const { return m->ncam; }
  int nlight() const { return m->nlight; }
  int nmesh() const { return m->nmesh; }
  int nmeshvert() const { return m->nmeshvert; }
  // int nmeshtexvert() const { return m->nmeshtexvert; }
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
  int nM() const { return m->nM; }
  int nD() const { return m->nD; }
  int nemax() const { return m->nemax; }
  int njmax() const { return m->njmax; }
  int nconmax() const { return m->nconmax; }
  // int nstack() const { return m->nstack; }
  int nuserdata() const { return m->nuserdata; }
  int nsensordata() const { return m->nsensordata; }
  int npluginstate() const { return m->npluginstate; }
  int nbuffer() const { return m->nbuffer; }

  emscripten::val qpos0() const {
    return val(typed_memory_view(m->nq * 1, m->qpos0));
  }
  emscripten::val qpos_spring() const {
    return val(typed_memory_view(m->nq * 1, m->qpos_spring));
  }
  emscripten::val body_parentid() const {
    return val(typed_memory_view(m->nbody * 1, m->body_parentid));
  }
  emscripten::val body_rootid() const {
    return val(typed_memory_view(m->nbody * 1, m->body_rootid));
  }
  emscripten::val body_weldid() const {
    return val(typed_memory_view(m->nbody * 1, m->body_weldid));
  }
  emscripten::val body_mocapid() const {
    return val(typed_memory_view(m->nbody * 1, m->body_mocapid));
  }
  emscripten::val body_jntnum() const {
    return val(typed_memory_view(m->nbody * 1, m->body_jntnum));
  }
  emscripten::val body_jntadr() const {
    return val(typed_memory_view(m->nbody * 1, m->body_jntadr));
  }
  emscripten::val body_dofnum() const {
    return val(typed_memory_view(m->nbody * 1, m->body_dofnum));
  }
  emscripten::val body_dofadr() const {
    return val(typed_memory_view(m->nbody * 1, m->body_dofadr));
  }
  emscripten::val body_geomnum() const {
    return val(typed_memory_view(m->nbody * 1, m->body_geomnum));
  }
  emscripten::val body_geomadr() const {
    return val(typed_memory_view(m->nbody * 1, m->body_geomadr));
  }
  emscripten::val body_simple() const {
    return val(typed_memory_view(m->nbody * 1, m->body_simple));
  }
  emscripten::val body_sameframe() const {
    return val(typed_memory_view(m->nbody * 1, m->body_sameframe));
  }
  emscripten::val body_pos() const {
    return val(typed_memory_view(m->nbody * 3, m->body_pos));
  }
  emscripten::val body_quat() const {
    return val(typed_memory_view(m->nbody * 4, m->body_quat));
  }
  emscripten::val body_ipos() const {
    return val(typed_memory_view(m->nbody * 3, m->body_ipos));
  }
  emscripten::val body_iquat() const {
    return val(typed_memory_view(m->nbody * 4, m->body_iquat));
  }
  emscripten::val body_mass() const {
    return val(typed_memory_view(m->nbody * 1, m->body_mass));
  }
  emscripten::val body_subtreemass() const {
    return val(typed_memory_view(m->nbody * 1, m->body_subtreemass));
  }
  emscripten::val body_inertia() const {
    return val(typed_memory_view(m->nbody * 3, m->body_inertia));
  }
  emscripten::val body_invweight0() const {
    return val(typed_memory_view(m->nbody * 2, m->body_invweight0));
  }
  emscripten::val body_gravcomp() const {
    return val(typed_memory_view(m->nbody * 1, m->body_gravcomp));
  }
  emscripten::val body_user() const {
    return val(typed_memory_view(m->nbody * m->nuser_body, m->body_user));
  }
  emscripten::val body_plugin() const {
    return val(typed_memory_view(m->nbody * 1, m->body_plugin));
  }
  emscripten::val jnt_type() const {
    return val(typed_memory_view(m->njnt * 1, m->jnt_type));
  }
  emscripten::val jnt_qposadr() const {
    return val(typed_memory_view(m->njnt * 1, m->jnt_qposadr));
  }
  emscripten::val jnt_dofadr() const {
    return val(typed_memory_view(m->njnt * 1, m->jnt_dofadr));
  }
  emscripten::val jnt_bodyid() const {
    return val(typed_memory_view(m->njnt * 1, m->jnt_bodyid));
  }
  emscripten::val jnt_group() const {
    return val(typed_memory_view(m->njnt * 1, m->jnt_group));
  }
  emscripten::val jnt_limited() const {
    return val(typed_memory_view(m->njnt * 1, m->jnt_limited));
  }
  emscripten::val jnt_solref() const {
    return val(typed_memory_view(m->njnt * mjNREF, m->jnt_solref));
  }
  emscripten::val jnt_solimp() const {
    return val(typed_memory_view(m->njnt * mjNIMP, m->jnt_solimp));
  }
  emscripten::val jnt_pos() const {
    return val(typed_memory_view(m->njnt * 3, m->jnt_pos));
  }
  emscripten::val jnt_axis() const {
    return val(typed_memory_view(m->njnt * 3, m->jnt_axis));
  }
  emscripten::val jnt_stiffness() const {
    return val(typed_memory_view(m->njnt * 1, m->jnt_stiffness));
  }
  emscripten::val jnt_range() const {
    return val(typed_memory_view(m->njnt * 2, m->jnt_range));
  }
  emscripten::val jnt_margin() const {
    return val(typed_memory_view(m->njnt * 1, m->jnt_margin));
  }
  emscripten::val jnt_user() const {
    return val(typed_memory_view(m->njnt * m->nuser_jnt, m->jnt_user));
  }
  emscripten::val dof_bodyid() const {
    return val(typed_memory_view(m->nv * 1, m->dof_bodyid));
  }
  emscripten::val dof_jntid() const {
    return val(typed_memory_view(m->nv * 1, m->dof_jntid));
  }
  emscripten::val dof_parentid() const {
    return val(typed_memory_view(m->nv * 1, m->dof_parentid));
  }
  emscripten::val dof_Madr() const {
    return val(typed_memory_view(m->nv * 1, m->dof_Madr));
  }
  emscripten::val dof_simplenum() const {
    return val(typed_memory_view(m->nv * 1, m->dof_simplenum));
  }
  emscripten::val dof_solref() const {
    return val(typed_memory_view(m->nv * mjNREF, m->dof_solref));
  }
  emscripten::val dof_solimp() const {
    return val(typed_memory_view(m->nv * mjNIMP, m->dof_solimp));
  }
  emscripten::val dof_frictionloss() const {
    return val(typed_memory_view(m->nv * 1, m->dof_frictionloss));
  }
  emscripten::val dof_armature() const {
    return val(typed_memory_view(m->nv * 1, m->dof_armature));
  }
  emscripten::val dof_damping() const {
    return val(typed_memory_view(m->nv * 1, m->dof_damping));
  }
  emscripten::val dof_invweight0() const {
    return val(typed_memory_view(m->nv * 1, m->dof_invweight0));
  }
  emscripten::val dof_M0() const {
    return val(typed_memory_view(m->nv * 1, m->dof_M0));
  }
  emscripten::val geom_type() const {
    return val(typed_memory_view(m->ngeom * 1, m->geom_type));
  }
  emscripten::val geom_contype() const {
    return val(typed_memory_view(m->ngeom * 1, m->geom_contype));
  }
  emscripten::val geom_conaffinity() const {
    return val(typed_memory_view(m->ngeom * 1, m->geom_conaffinity));
  }
  emscripten::val geom_condim() const {
    return val(typed_memory_view(m->ngeom * 1, m->geom_condim));
  }
  emscripten::val geom_bodyid() const {
    return val(typed_memory_view(m->ngeom * 1, m->geom_bodyid));
  }
  emscripten::val geom_dataid() const {
    return val(typed_memory_view(m->ngeom * 1, m->geom_dataid));
  }
  emscripten::val geom_matid() const {
    return val(typed_memory_view(m->ngeom * 1, m->geom_matid));
  }
  emscripten::val geom_group() const {
    return val(typed_memory_view(m->ngeom * 1, m->geom_group));
  }
  emscripten::val geom_priority() const {
    return val(typed_memory_view(m->ngeom * 1, m->geom_priority));
  }
  emscripten::val geom_sameframe() const {
    return val(typed_memory_view(m->ngeom * 1, m->geom_sameframe));
  }
  emscripten::val geom_solmix() const {
    return val(typed_memory_view(m->ngeom * 1, m->geom_solmix));
  }
  emscripten::val geom_solref() const {
    return val(typed_memory_view(m->ngeom * mjNREF, m->geom_solref));
  }
  emscripten::val geom_solimp() const {
    return val(typed_memory_view(m->ngeom * mjNIMP, m->geom_solimp));
  }
  emscripten::val geom_size() const {
    return val(typed_memory_view(m->ngeom * 3, m->geom_size));
  }
  emscripten::val geom_rbound() const {
    return val(typed_memory_view(m->ngeom * 1, m->geom_rbound));
  }
  emscripten::val geom_pos() const {
    return val(typed_memory_view(m->ngeom * 3, m->geom_pos));
  }
  emscripten::val geom_quat() const {
    return val(typed_memory_view(m->ngeom * 4, m->geom_quat));
  }
  emscripten::val geom_friction() const {
    return val(typed_memory_view(m->ngeom * 3, m->geom_friction));
  }
  emscripten::val geom_margin() const {
    return val(typed_memory_view(m->ngeom * 1, m->geom_margin));
  }
  emscripten::val geom_gap() const {
    return val(typed_memory_view(m->ngeom * 1, m->geom_gap));
  }
  emscripten::val geom_fluid() const {
    return val(typed_memory_view(m->ngeom * mjNFLUID, m->geom_fluid));
  }
  emscripten::val geom_user() const {
    return val(typed_memory_view(m->ngeom * m->nuser_geom, m->geom_user));
  }
  emscripten::val geom_rgba() const {
    return val(typed_memory_view(m->ngeom * 4, m->geom_rgba));
  }
  emscripten::val site_type() const {
    return val(typed_memory_view(m->nsite * 1, m->site_type));
  }
  emscripten::val site_bodyid() const {
    return val(typed_memory_view(m->nsite * 1, m->site_bodyid));
  }
  emscripten::val site_matid() const {
    return val(typed_memory_view(m->nsite * 1, m->site_matid));
  }
  emscripten::val site_group() const {
    return val(typed_memory_view(m->nsite * 1, m->site_group));
  }
  emscripten::val site_sameframe() const {
    return val(typed_memory_view(m->nsite * 1, m->site_sameframe));
  }
  emscripten::val site_size() const {
    return val(typed_memory_view(m->nsite * 3, m->site_size));
  }
  emscripten::val site_pos() const {
    return val(typed_memory_view(m->nsite * 3, m->site_pos));
  }
  emscripten::val site_quat() const {
    return val(typed_memory_view(m->nsite * 4, m->site_quat));
  }
  emscripten::val site_user() const {
    return val(typed_memory_view(m->nsite * m->nuser_site, m->site_user));
  }
  emscripten::val site_rgba() const {
    return val(typed_memory_view(m->nsite * 4, m->site_rgba));
  }
  emscripten::val cam_mode() const {
    return val(typed_memory_view(m->ncam * 1, m->cam_mode));
  }
  emscripten::val cam_bodyid() const {
    return val(typed_memory_view(m->ncam * 1, m->cam_bodyid));
  }
  emscripten::val cam_targetbodyid() const {
    return val(typed_memory_view(m->ncam * 1, m->cam_targetbodyid));
  }
  emscripten::val cam_pos() const {
    return val(typed_memory_view(m->ncam * 3, m->cam_pos));
  }
  emscripten::val cam_quat() const {
    return val(typed_memory_view(m->ncam * 4, m->cam_quat));
  }
  emscripten::val cam_poscom0() const {
    return val(typed_memory_view(m->ncam * 3, m->cam_poscom0));
  }
  emscripten::val cam_pos0() const {
    return val(typed_memory_view(m->ncam * 3, m->cam_pos0));
  }
  emscripten::val cam_mat0() const {
    return val(typed_memory_view(m->ncam * 9, m->cam_mat0));
  }
  emscripten::val cam_fovy() const {
    return val(typed_memory_view(m->ncam * 1, m->cam_fovy));
  }
  emscripten::val cam_ipd() const {
    return val(typed_memory_view(m->ncam * 1, m->cam_ipd));
  }
  emscripten::val cam_user() const {
    return val(typed_memory_view(m->ncam * m->nuser_cam, m->cam_user));
  }
  emscripten::val light_mode() const {
    return val(typed_memory_view(m->nlight * 1, m->light_mode));
  }
  emscripten::val light_bodyid() const {
    return val(typed_memory_view(m->nlight * 1, m->light_bodyid));
  }
  emscripten::val light_targetbodyid() const {
    return val(typed_memory_view(m->nlight * 1, m->light_targetbodyid));
  }
  emscripten::val light_directional() const {
    return val(typed_memory_view(m->nlight * 1, m->light_directional));
  }
  emscripten::val light_castshadow() const {
    return val(typed_memory_view(m->nlight * 1, m->light_castshadow));
  }
  emscripten::val light_active() const {
    return val(typed_memory_view(m->nlight * 1, m->light_active));
  }
  emscripten::val light_pos() const {
    return val(typed_memory_view(m->nlight * 3, m->light_pos));
  }
  emscripten::val light_dir() const {
    return val(typed_memory_view(m->nlight * 3, m->light_dir));
  }
  emscripten::val light_poscom0() const {
    return val(typed_memory_view(m->nlight * 3, m->light_poscom0));
  }
  emscripten::val light_pos0() const {
    return val(typed_memory_view(m->nlight * 3, m->light_pos0));
  }
  emscripten::val light_dir0() const {
    return val(typed_memory_view(m->nlight * 3, m->light_dir0));
  }
  emscripten::val light_attenuation() const {
    return val(typed_memory_view(m->nlight * 3, m->light_attenuation));
  }
  emscripten::val light_cutoff() const {
    return val(typed_memory_view(m->nlight * 1, m->light_cutoff));
  }
  emscripten::val light_exponent() const {
    return val(typed_memory_view(m->nlight * 1, m->light_exponent));
  }
  emscripten::val light_ambient() const {
    return val(typed_memory_view(m->nlight * 3, m->light_ambient));
  }
  emscripten::val light_diffuse() const {
    return val(typed_memory_view(m->nlight * 3, m->light_diffuse));
  }
  emscripten::val light_specular() const {
    return val(typed_memory_view(m->nlight * 3, m->light_specular));
  }
  emscripten::val mesh_vertadr() const {
    return val(typed_memory_view(m->nmesh * 1, m->mesh_vertadr));
  }
  emscripten::val mesh_vertnum() const {
    return val(typed_memory_view(m->nmesh * 1, m->mesh_vertnum));
  }
  emscripten::val mesh_texcoordadr() const {
    return val(typed_memory_view(m->nmesh * 1, m->mesh_texcoordadr));
  }
  emscripten::val mesh_faceadr() const {
    return val(typed_memory_view(m->nmesh * 1, m->mesh_faceadr));
  }
  emscripten::val mesh_facenum() const {
    return val(typed_memory_view(m->nmesh * 1, m->mesh_facenum));
  }
  emscripten::val mesh_graphadr() const {
    return val(typed_memory_view(m->nmesh * 1, m->mesh_graphadr));
  }
  emscripten::val mesh_vert() const {
    return val(typed_memory_view(m->nmeshvert * 3, m->mesh_vert));
  }
  emscripten::val mesh_normal() const {
    return val(typed_memory_view(m->nmeshvert * 3, m->mesh_normal));
  }
  // emscripten::val mesh_texcoord() const {
  //   return val(typed_memory_view(m->nmeshtexvert * 2, m->mesh_texcoord));
  // }
  emscripten::val mesh_face() const {
    return val(typed_memory_view(m->nmeshface * 3, m->mesh_face));
  }
  emscripten::val mesh_graph() const {
    return val(typed_memory_view(m->nmeshgraph * 1, m->mesh_graph));
  }
  emscripten::val skin_matid() const {
    return val(typed_memory_view(m->nskin * 1, m->skin_matid));
  }
  emscripten::val skin_group() const {
    return val(typed_memory_view(m->nskin * 1, m->skin_group));
  }
  emscripten::val skin_rgba() const {
    return val(typed_memory_view(m->nskin * 4, m->skin_rgba));
  }
  emscripten::val skin_inflate() const {
    return val(typed_memory_view(m->nskin * 1, m->skin_inflate));
  }
  emscripten::val skin_vertadr() const {
    return val(typed_memory_view(m->nskin * 1, m->skin_vertadr));
  }
  emscripten::val skin_vertnum() const {
    return val(typed_memory_view(m->nskin * 1, m->skin_vertnum));
  }
  emscripten::val skin_texcoordadr() const {
    return val(typed_memory_view(m->nskin * 1, m->skin_texcoordadr));
  }
  emscripten::val skin_faceadr() const {
    return val(typed_memory_view(m->nskin * 1, m->skin_faceadr));
  }
  emscripten::val skin_facenum() const {
    return val(typed_memory_view(m->nskin * 1, m->skin_facenum));
  }
  emscripten::val skin_boneadr() const {
    return val(typed_memory_view(m->nskin * 1, m->skin_boneadr));
  }
  emscripten::val skin_bonenum() const {
    return val(typed_memory_view(m->nskin * 1, m->skin_bonenum));
  }
  emscripten::val skin_vert() const {
    return val(typed_memory_view(m->nskinvert * 3, m->skin_vert));
  }
  emscripten::val skin_texcoord() const {
    return val(typed_memory_view(m->nskintexvert * 2, m->skin_texcoord));
  }
  emscripten::val skin_face() const {
    return val(typed_memory_view(m->nskinface * 3, m->skin_face));
  }
  emscripten::val skin_bonevertadr() const {
    return val(typed_memory_view(m->nskinbone * 1, m->skin_bonevertadr));
  }
  emscripten::val skin_bonevertnum() const {
    return val(typed_memory_view(m->nskinbone * 1, m->skin_bonevertnum));
  }
  emscripten::val skin_bonebindpos() const {
    return val(typed_memory_view(m->nskinbone * 3, m->skin_bonebindpos));
  }
  emscripten::val skin_bonebindquat() const {
    return val(typed_memory_view(m->nskinbone * 4, m->skin_bonebindquat));
  }
  emscripten::val skin_bonebodyid() const {
    return val(typed_memory_view(m->nskinbone * 1, m->skin_bonebodyid));
  }
  emscripten::val skin_bonevertid() const {
    return val(typed_memory_view(m->nskinbonevert * 1, m->skin_bonevertid));
  }
  emscripten::val skin_bonevertweight() const {
    return val(typed_memory_view(m->nskinbonevert * 1, m->skin_bonevertweight));
  }
  emscripten::val hfield_size() const {
    return val(typed_memory_view(m->nhfield * 4, m->hfield_size));
  }
  emscripten::val hfield_nrow() const {
    return val(typed_memory_view(m->nhfield * 1, m->hfield_nrow));
  }
  emscripten::val hfield_ncol() const {
    return val(typed_memory_view(m->nhfield * 1, m->hfield_ncol));
  }
  emscripten::val hfield_adr() const {
    return val(typed_memory_view(m->nhfield * 1, m->hfield_adr));
  }
  emscripten::val hfield_data() const {
    return val(typed_memory_view(m->nhfielddata * 1, m->hfield_data));
  }
  emscripten::val tex_type() const {
    return val(typed_memory_view(m->ntex * 1, m->tex_type));
  }
  emscripten::val tex_height() const {
    return val(typed_memory_view(m->ntex * 1, m->tex_height));
  }
  emscripten::val tex_width() const {
    return val(typed_memory_view(m->ntex * 1, m->tex_width));
  }
  emscripten::val tex_adr() const {
    return val(typed_memory_view(m->ntex * 1, m->tex_adr));
  }
  // emscripten::val tex_rgb() const {
  //   return val(typed_memory_view(m->ntexdata * 1, m->tex_rgb));
  // }
  emscripten::val mat_texid() const {
    return val(typed_memory_view(m->nmat * 1, m->mat_texid));
  }
  emscripten::val mat_texuniform() const {
    return val(typed_memory_view(m->nmat * 1, m->mat_texuniform));
  }
  emscripten::val mat_texrepeat() const {
    return val(typed_memory_view(m->nmat * 2, m->mat_texrepeat));
  }
  emscripten::val mat_emission() const {
    return val(typed_memory_view(m->nmat * 1, m->mat_emission));
  }
  emscripten::val mat_specular() const {
    return val(typed_memory_view(m->nmat * 1, m->mat_specular));
  }
  emscripten::val mat_shininess() const {
    return val(typed_memory_view(m->nmat * 1, m->mat_shininess));
  }
  emscripten::val mat_reflectance() const {
    return val(typed_memory_view(m->nmat * 1, m->mat_reflectance));
  }
  emscripten::val mat_rgba() const {
    return val(typed_memory_view(m->nmat * 4, m->mat_rgba));
  }
  emscripten::val pair_dim() const {
    return val(typed_memory_view(m->npair * 1, m->pair_dim));
  }
  emscripten::val pair_geom1() const {
    return val(typed_memory_view(m->npair * 1, m->pair_geom1));
  }
  emscripten::val pair_geom2() const {
    return val(typed_memory_view(m->npair * 1, m->pair_geom2));
  }
  emscripten::val pair_signature() const {
    return val(typed_memory_view(m->npair * 1, m->pair_signature));
  }
  emscripten::val pair_solref() const {
    return val(typed_memory_view(m->npair * mjNREF, m->pair_solref));
  }
  emscripten::val pair_solimp() const {
    return val(typed_memory_view(m->npair * mjNIMP, m->pair_solimp));
  }
  emscripten::val pair_margin() const {
    return val(typed_memory_view(m->npair * 1, m->pair_margin));
  }
  emscripten::val pair_gap() const {
    return val(typed_memory_view(m->npair * 1, m->pair_gap));
  }
  emscripten::val pair_friction() const {
    return val(typed_memory_view(m->npair * 5, m->pair_friction));
  }
  emscripten::val exclude_signature() const {
    return val(typed_memory_view(m->nexclude * 1, m->exclude_signature));
  }
  emscripten::val eq_type() const {
    return val(typed_memory_view(m->neq * 1, m->eq_type));
  }
  emscripten::val eq_obj1id() const {
    return val(typed_memory_view(m->neq * 1, m->eq_obj1id));
  }
  emscripten::val eq_obj2id() const {
    return val(typed_memory_view(m->neq * 1, m->eq_obj2id));
  }
  // emscripten::val eq_active() const {
  //   return val(typed_memory_view(m->neq * 1, m->eq_active));
  // }
  emscripten::val eq_solref() const {
    return val(typed_memory_view(m->neq * mjNREF, m->eq_solref));
  }
  emscripten::val eq_solimp() const {
    return val(typed_memory_view(m->neq * mjNIMP, m->eq_solimp));
  }
  emscripten::val eq_data() const {
    return val(typed_memory_view(m->neq * mjNEQDATA, m->eq_data));
  }
  emscripten::val tendon_adr() const {
    return val(typed_memory_view(m->ntendon * 1, m->tendon_adr));
  }
  emscripten::val tendon_num() const {
    return val(typed_memory_view(m->ntendon * 1, m->tendon_num));
  }
  emscripten::val tendon_matid() const {
    return val(typed_memory_view(m->ntendon * 1, m->tendon_matid));
  }
  emscripten::val tendon_group() const {
    return val(typed_memory_view(m->ntendon * 1, m->tendon_group));
  }
  emscripten::val tendon_limited() const {
    return val(typed_memory_view(m->ntendon * 1, m->tendon_limited));
  }
  emscripten::val tendon_width() const {
    return val(typed_memory_view(m->ntendon * 1, m->tendon_width));
  }
  emscripten::val tendon_solref_lim() const {
    return val(typed_memory_view(m->ntendon * mjNREF, m->tendon_solref_lim));
  }
  emscripten::val tendon_solimp_lim() const {
    return val(typed_memory_view(m->ntendon * mjNIMP, m->tendon_solimp_lim));
  }
  emscripten::val tendon_solref_fri() const {
    return val(typed_memory_view(m->ntendon * mjNREF, m->tendon_solref_fri));
  }
  emscripten::val tendon_solimp_fri() const {
    return val(typed_memory_view(m->ntendon * mjNIMP, m->tendon_solimp_fri));
  }
  emscripten::val tendon_range() const {
    return val(typed_memory_view(m->ntendon * 2, m->tendon_range));
  }
  emscripten::val tendon_margin() const {
    return val(typed_memory_view(m->ntendon * 1, m->tendon_margin));
  }
  emscripten::val tendon_stiffness() const {
    return val(typed_memory_view(m->ntendon * 1, m->tendon_stiffness));
  }
  emscripten::val tendon_damping() const {
    return val(typed_memory_view(m->ntendon * 1, m->tendon_damping));
  }
  emscripten::val tendon_frictionloss() const {
    return val(typed_memory_view(m->ntendon * 1, m->tendon_frictionloss));
  }
  emscripten::val tendon_lengthspring() const {
    return val(typed_memory_view(m->ntendon * 2, m->tendon_lengthspring));
  }
  emscripten::val tendon_length0() const {
    return val(typed_memory_view(m->ntendon * 1, m->tendon_length0));
  }
  emscripten::val tendon_invweight0() const {
    return val(typed_memory_view(m->ntendon * 1, m->tendon_invweight0));
  }
  emscripten::val tendon_user() const {
    return val(typed_memory_view(m->ntendon * m->nuser_tendon, m->tendon_user));
  }
  emscripten::val tendon_rgba() const {
    return val(typed_memory_view(m->ntendon * 4, m->tendon_rgba));
  }
  emscripten::val wrap_type() const {
    return val(typed_memory_view(m->nwrap * 1, m->wrap_type));
  }
  emscripten::val wrap_objid() const {
    return val(typed_memory_view(m->nwrap * 1, m->wrap_objid));
  }
  emscripten::val wrap_prm() const {
    return val(typed_memory_view(m->nwrap * 1, m->wrap_prm));
  }
  emscripten::val actuator_trntype() const {
    return val(typed_memory_view(m->nu * 1, m->actuator_trntype));
  }
  emscripten::val actuator_dyntype() const {
    return val(typed_memory_view(m->nu * 1, m->actuator_dyntype));
  }
  emscripten::val actuator_gaintype() const {
    return val(typed_memory_view(m->nu * 1, m->actuator_gaintype));
  }
  emscripten::val actuator_biastype() const {
    return val(typed_memory_view(m->nu * 1, m->actuator_biastype));
  }
  emscripten::val actuator_trnid() const {
    return val(typed_memory_view(m->nu * 2, m->actuator_trnid));
  }
  emscripten::val actuator_actadr() const {
    return val(typed_memory_view(m->nu * 1, m->actuator_actadr));
  }
  emscripten::val actuator_actnum() const {
    return val(typed_memory_view(m->nu * 1, m->actuator_actnum));
  }
  emscripten::val actuator_group() const {
    return val(typed_memory_view(m->nu * 1, m->actuator_group));
  }
  emscripten::val actuator_ctrllimited() const {
    return val(typed_memory_view(m->nu * 1, m->actuator_ctrllimited));
  }
  emscripten::val actuator_forcelimited() const {
    return val(typed_memory_view(m->nu * 1, m->actuator_forcelimited));
  }
  emscripten::val actuator_actlimited() const {
    return val(typed_memory_view(m->nu * 1, m->actuator_actlimited));
  }
  emscripten::val actuator_dynprm() const {
    return val(typed_memory_view(m->nu * mjNDYN, m->actuator_dynprm));
  }
  emscripten::val actuator_gainprm() const {
    return val(typed_memory_view(m->nu * mjNGAIN, m->actuator_gainprm));
  }
  emscripten::val actuator_biasprm() const {
    return val(typed_memory_view(m->nu * mjNBIAS, m->actuator_biasprm));
  }
  emscripten::val actuator_ctrlrange() const {
    return val(typed_memory_view(m->nu * 2, m->actuator_ctrlrange));
  }
  emscripten::val actuator_forcerange() const {
    return val(typed_memory_view(m->nu * 2, m->actuator_forcerange));
  }
  emscripten::val actuator_actrange() const {
    return val(typed_memory_view(m->nu * 2, m->actuator_actrange));
  }
  emscripten::val actuator_gear() const {
    return val(typed_memory_view(m->nu * 6, m->actuator_gear));
  }
  emscripten::val actuator_cranklength() const {
    return val(typed_memory_view(m->nu * 1, m->actuator_cranklength));
  }
  emscripten::val actuator_acc0() const {
    return val(typed_memory_view(m->nu * 1, m->actuator_acc0));
  }
  emscripten::val actuator_length0() const {
    return val(typed_memory_view(m->nu * 1, m->actuator_length0));
  }
  emscripten::val actuator_lengthrange() const {
    return val(typed_memory_view(m->nu * 2, m->actuator_lengthrange));
  }
  emscripten::val actuator_user() const {
    return val(typed_memory_view(m->nu * m->nuser_actuator, m->actuator_user));
  }
  emscripten::val actuator_plugin() const {
    return val(typed_memory_view(m->nu * 1, m->actuator_plugin));
  }
  emscripten::val sensor_type() const {
    return val(typed_memory_view(m->nsensor * 1, m->sensor_type));
  }
  emscripten::val sensor_datatype() const {
    return val(typed_memory_view(m->nsensor * 1, m->sensor_datatype));
  }
  emscripten::val sensor_needstage() const {
    return val(typed_memory_view(m->nsensor * 1, m->sensor_needstage));
  }
  emscripten::val sensor_objtype() const {
    return val(typed_memory_view(m->nsensor * 1, m->sensor_objtype));
  }
  emscripten::val sensor_objid() const {
    return val(typed_memory_view(m->nsensor * 1, m->sensor_objid));
  }
  emscripten::val sensor_reftype() const {
    return val(typed_memory_view(m->nsensor * 1, m->sensor_reftype));
  }
  emscripten::val sensor_refid() const {
    return val(typed_memory_view(m->nsensor * 1, m->sensor_refid));
  }
  emscripten::val sensor_dim() const {
    return val(typed_memory_view(m->nsensor * 1, m->sensor_dim));
  }
  emscripten::val sensor_adr() const {
    return val(typed_memory_view(m->nsensor * 1, m->sensor_adr));
  }
  emscripten::val sensor_cutoff() const {
    return val(typed_memory_view(m->nsensor * 1, m->sensor_cutoff));
  }
  emscripten::val sensor_noise() const {
    return val(typed_memory_view(m->nsensor * 1, m->sensor_noise));
  }
  emscripten::val sensor_user() const {
    return val(typed_memory_view(m->nsensor * m->nuser_sensor, m->sensor_user));
  }
  emscripten::val sensor_plugin() const {
    return val(typed_memory_view(m->nsensor * 1, m->sensor_plugin));
  }
  emscripten::val plugin() const {
    return val(typed_memory_view(m->nplugin * 1, m->plugin));
  }
  emscripten::val plugin_stateadr() const {
    return val(typed_memory_view(m->nplugin * 1, m->plugin_stateadr));
  }
  emscripten::val plugin_statenum() const {
    return val(typed_memory_view(m->nplugin * 1, m->plugin_statenum));
  }
  emscripten::val plugin_attr() const {
    return val(typed_memory_view(m->npluginattr * 1, m->plugin_attr));
  }
  emscripten::val plugin_attradr() const {
    return val(typed_memory_view(m->nplugin * 1, m->plugin_attradr));
  }
  emscripten::val numeric_adr() const {
    return val(typed_memory_view(m->nnumeric * 1, m->numeric_adr));
  }
  emscripten::val numeric_size() const {
    return val(typed_memory_view(m->nnumeric * 1, m->numeric_size));
  }
  emscripten::val numeric_data() const {
    return val(typed_memory_view(m->nnumericdata * 1, m->numeric_data));
  }
  emscripten::val text_adr() const {
    return val(typed_memory_view(m->ntext * 1, m->text_adr));
  }
  emscripten::val text_size() const {
    return val(typed_memory_view(m->ntext * 1, m->text_size));
  }
  emscripten::val text_data() const {
    return val(typed_memory_view(m->ntextdata * 1, m->text_data));
  }
  emscripten::val tuple_adr() const {
    return val(typed_memory_view(m->ntuple * 1, m->tuple_adr));
  }
  emscripten::val tuple_size() const {
    return val(typed_memory_view(m->ntuple * 1, m->tuple_size));
  }
  emscripten::val tuple_objtype() const {
    return val(typed_memory_view(m->ntupledata * 1, m->tuple_objtype));
  }
  emscripten::val tuple_objid() const {
    return val(typed_memory_view(m->ntupledata * 1, m->tuple_objid));
  }
  emscripten::val tuple_objprm() const {
    return val(typed_memory_view(m->ntupledata * 1, m->tuple_objprm));
  }
  emscripten::val key_time() const {
    return val(typed_memory_view(m->nkey * 1, m->key_time));
  }
  emscripten::val key_qpos() const {
    return val(typed_memory_view(m->nkey * m->nq, m->key_qpos));
  }
  emscripten::val key_qvel() const {
    return val(typed_memory_view(m->nkey * m->nv, m->key_qvel));
  }
  emscripten::val key_act() const {
    return val(typed_memory_view(m->nkey * m->na, m->key_act));
  }
  emscripten::val key_mpos() const {
    return val(typed_memory_view(m->nkey * m->nmocap, m->key_mpos));
  }
  emscripten::val key_mquat() const {
    return val(typed_memory_view(m->nkey * m->nmocap, m->key_mquat));
  }
  emscripten::val key_ctrl() const {
    return val(typed_memory_view(m->nkey * m->nu, m->key_ctrl));
  }
  emscripten::val name_bodyadr() const {
    return val(typed_memory_view(m->nbody * 1, m->name_bodyadr));
  }
  emscripten::val name_jntadr() const {
    return val(typed_memory_view(m->njnt * 1, m->name_jntadr));
  }
  emscripten::val name_geomadr() const {
    return val(typed_memory_view(m->ngeom * 1, m->name_geomadr));
  }
  emscripten::val name_siteadr() const {
    return val(typed_memory_view(m->nsite * 1, m->name_siteadr));
  }
  emscripten::val name_camadr() const {
    return val(typed_memory_view(m->ncam * 1, m->name_camadr));
  }
  emscripten::val name_lightadr() const {
    return val(typed_memory_view(m->nlight * 1, m->name_lightadr));
  }
  emscripten::val name_meshadr() const {
    return val(typed_memory_view(m->nmesh * 1, m->name_meshadr));
  }
  emscripten::val name_skinadr() const {
    return val(typed_memory_view(m->nskin * 1, m->name_skinadr));
  }
  emscripten::val name_hfieldadr() const {
    return val(typed_memory_view(m->nhfield * 1, m->name_hfieldadr));
  }
  emscripten::val name_texadr() const {
    return val(typed_memory_view(m->ntex * 1, m->name_texadr));
  }
  emscripten::val name_matadr() const {
    return val(typed_memory_view(m->nmat * 1, m->name_matadr));
  }
  emscripten::val name_pairadr() const {
    return val(typed_memory_view(m->npair * 1, m->name_pairadr));
  }
  emscripten::val name_excludeadr() const {
    return val(typed_memory_view(m->nexclude * 1, m->name_excludeadr));
  }
  emscripten::val name_eqadr() const {
    return val(typed_memory_view(m->neq * 1, m->name_eqadr));
  }
  emscripten::val name_tendonadr() const {
    return val(typed_memory_view(m->ntendon * 1, m->name_tendonadr));
  }
  emscripten::val name_actuatoradr() const {
    return val(typed_memory_view(m->nu * 1, m->name_actuatoradr));
  }
  emscripten::val name_sensoradr() const {
    return val(typed_memory_view(m->nsensor * 1, m->name_sensoradr));
  }
  emscripten::val name_numericadr() const {
    return val(typed_memory_view(m->nnumeric * 1, m->name_numericadr));
  }
  emscripten::val name_textadr() const {
    return val(typed_memory_view(m->ntext * 1, m->name_textadr));
  }
  emscripten::val name_tupleadr() const {
    return val(typed_memory_view(m->ntuple * 1, m->name_tupleadr));
  }
  emscripten::val name_keyadr() const {
    return val(typed_memory_view(m->nkey * 1, m->name_keyadr));
  }
  emscripten::val name_pluginadr() const {
    return val(typed_memory_view(m->nplugin * 1, m->name_pluginadr));
  }
  emscripten::val names() const {
    return val(typed_memory_view(m->nnames * 1, m->names));
  }

private:
  mjModel *m;
};

class State {
public:
  State(Model m) { d = mj_makeData(m.ptr()); }
  mjData *ptr() { return d; }
  mjData getVal() { return *d; }
  void free() { return mju_free(d); }

private:
  mjData *d;
};

class Simulation {
public:
  Simulation(Model *m, State *s) {
    _model = m;
    _state = s;
  }

  State *state() { return _state; }
  Model *model() { return _model; }
  void free() {
    mju_free(_state);
    mju_free(_model);
  }

  void applyForce(mjtNum fx, mjtNum fy, mjtNum fz, mjtNum tx, mjtNum ty,
                  mjtNum tz, mjtNum px, mjtNum py, mjtNum pz, int body) {
    mjtNum force[3] = {fx, fy, fz};
    mjtNum torque[3] = {tx, ty, tz};
    mjtNum point[3] = {px, py, pz};
    mj_applyFT(_model->ptr(), _state->ptr(), force, torque, point, body,
               _state->ptr()->qfrc_applied);
  }

  // copied from the source of mjv_applyPerturbPose
  // sets perturb pos,quat in d->mocap when selected body is mocap, and in
  // d->qpos otherwise
  //  d->qpos written only if flg_paused and subtree root for selected body has
  //  free joint
  void applyPose(int bodyID, mjtNum refPosX, mjtNum refPosY, mjtNum refPosZ,
                 mjtNum refQuat1, mjtNum refQuat2, mjtNum refQuat3,
                 mjtNum refQuat4, int flg_paused) {
    int rootid = 0, sel = bodyID; // pert->select;
    mjtNum pos1[3], quat1[4], pos2[3], quat2[4], refpos[3], refquat[4];
    mjtNum *Rpos, *Rquat, *Cpos, *Cquat;
    mjtNum inrefpos[3] = {refPosX, refPosY, refPosZ};
    mjtNum inrefquat[4] = {refQuat1, refQuat2, refQuat3, refQuat4};
    mjModel *m = _model->ptr();
    mjData *d = _state->ptr();

    // exit if nothing to do
    // if (sel<=0 || sel>=m->nbody || !(pert->active | pert->active2)) { return;
    // }

    // get rootid above selected body
    rootid = m->body_rootid[sel];

    // transform refpos,refquat from I-frame to X-frame of body[sel]
    mju_negPose(pos1, quat1, m->body_ipos + 3 * sel, m->body_iquat + 4 * sel);
    mju_mulPose(refpos, refquat, inrefpos, inrefquat, pos1, quat1);

    // mocap body
    if (m->body_mocapid[sel] >= 0) {
      // copy ref pose into mocap pose
      mju_copy3(d->mocap_pos + 3 * m->body_mocapid[sel], refpos);
      mju_copy4(d->mocap_quat + 4 * m->body_mocapid[sel], refquat);
    }

    // floating body, paused
    else if (flg_paused && m->body_jntnum[sel] == 1 &&
             m->jnt_type[m->body_jntadr[sel]] == mjJNT_FREE) {
      // copy ref pose into qpos
      mju_copy3(d->qpos + m->jnt_qposadr[m->body_jntadr[sel]], refpos);
      mju_copy4(d->qpos + m->jnt_qposadr[m->body_jntadr[sel]] + 3, refquat);
    }

    // child of floating body, paused
    else if (flg_paused && m->body_jntnum[rootid] == 1 &&
             m->jnt_type[m->body_jntadr[rootid]] == mjJNT_FREE) {
      // get pointers to root
      Rpos = d->qpos + m->jnt_qposadr[m->body_jntadr[rootid]];
      Rquat = Rpos + 3;

      // get pointers to child
      Cpos = d->xpos + 3 * sel;
      Cquat = d->xquat + 4 * sel;

      // set root <- ref*neg(child)*root
      mju_negPose(pos1, quat1, Cpos, Cquat);              // neg(child)
      mju_mulPose(pos2, quat2, pos1, quat1, Rpos, Rquat); // neg(child)*root
      mju_mulPose(Rpos, Rquat, refpos, refquat, pos2,
                  quat2); // ref*neg(child)*root
    }
  }

  // MJDATA_DEFINITIONS
  val qpos() const {
    return val(typed_memory_view(_model->ptr()->nq * 1, _state->ptr()->qpos));
  }
  val qvel() const {
    return val(typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qvel));
  }
  val act() const {
    return val(typed_memory_view(_model->ptr()->na * 1, _state->ptr()->act));
  }
  val qacc_warmstart() const {
    return val(typed_memory_view(_model->ptr()->nv * 1,
                                 _state->ptr()->qacc_warmstart));
  }
  val plugin_state() const {
    return val(typed_memory_view(_model->ptr()->npluginstate * 1,
                                 _state->ptr()->plugin_state));
  }
  val ctrl() const {
    return val(typed_memory_view(_model->ptr()->nu * 1, _state->ptr()->ctrl));
  }
  val qfrc_applied() const {
    return val(
        typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qfrc_applied));
  }
  val xfrc_applied() const {
    return val(typed_memory_view(_model->ptr()->nbody * 6,
                                 _state->ptr()->xfrc_applied));
  }
  val mocap_pos() const {
    return val(
        typed_memory_view(_model->ptr()->nmocap * 3, _state->ptr()->mocap_pos));
  }
  val mocap_quat() const {
    return val(typed_memory_view(_model->ptr()->nmocap * 4,
                                 _state->ptr()->mocap_quat));
  }
  val qacc() const {
    return val(typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qacc));
  }
  val act_dot() const {
    return val(
        typed_memory_view(_model->ptr()->na * 1, _state->ptr()->act_dot));
  }
  val userdata() const {
    return val(typed_memory_view(_model->ptr()->nuserdata * 1,
                                 _state->ptr()->userdata));
  }
  val sensordata() const {
    return val(typed_memory_view(_model->ptr()->nsensordata * 1,
                                 _state->ptr()->sensordata));
  }
  val plugin() const {
    return val(
        typed_memory_view(_model->ptr()->nplugin * 1, _state->ptr()->plugin));
  }
  val plugin_data() const {
    return val(typed_memory_view(_model->ptr()->nplugin * 1,
                                 _state->ptr()->plugin_data));
  }
  val xpos() const {
    return val(
        typed_memory_view(_model->ptr()->nbody * 3, _state->ptr()->xpos));
  }
  val xquat() const {
    return val(
        typed_memory_view(_model->ptr()->nbody * 4, _state->ptr()->xquat));
  }
  val xmat() const {
    return val(
        typed_memory_view(_model->ptr()->nbody * 9, _state->ptr()->xmat));
  }
  val xipos() const {
    return val(
        typed_memory_view(_model->ptr()->nbody * 3, _state->ptr()->xipos));
  }
  val ximat() const {
    return val(
        typed_memory_view(_model->ptr()->nbody * 9, _state->ptr()->ximat));
  }
  val xanchor() const {
    return val(
        typed_memory_view(_model->ptr()->njnt * 3, _state->ptr()->xanchor));
  }
  val xaxis() const {
    return val(
        typed_memory_view(_model->ptr()->njnt * 3, _state->ptr()->xaxis));
  }
  val geom_xpos() const {
    return val(
        typed_memory_view(_model->ptr()->ngeom * 3, _state->ptr()->geom_xpos));
  }
  val geom_xmat() const {
    return val(
        typed_memory_view(_model->ptr()->ngeom * 9, _state->ptr()->geom_xmat));
  }
  val site_xpos() const {
    return val(
        typed_memory_view(_model->ptr()->nsite * 3, _state->ptr()->site_xpos));
  }
  val site_xmat() const {
    return val(
        typed_memory_view(_model->ptr()->nsite * 9, _state->ptr()->site_xmat));
  }
  val cam_xpos() const {
    return val(
        typed_memory_view(_model->ptr()->ncam * 3, _state->ptr()->cam_xpos));
  }
  val cam_xmat() const {
    return val(
        typed_memory_view(_model->ptr()->ncam * 9, _state->ptr()->cam_xmat));
  }
  val light_xpos() const {
    return val(typed_memory_view(_model->ptr()->nlight * 3,
                                 _state->ptr()->light_xpos));
  }
  val light_xdir() const {
    return val(typed_memory_view(_model->ptr()->nlight * 3,
                                 _state->ptr()->light_xdir));
  }
  val subtree_com() const {
    return val(typed_memory_view(_model->ptr()->nbody * 3,
                                 _state->ptr()->subtree_com));
  }
  val cdof() const {
    return val(typed_memory_view(_model->ptr()->nv * 6, _state->ptr()->cdof));
  }
  val cinert() const {
    return val(
        typed_memory_view(_model->ptr()->nbody * 10, _state->ptr()->cinert));
  }
  val ten_wrapadr() const {
    return val(typed_memory_view(_model->ptr()->ntendon * 1,
                                 _state->ptr()->ten_wrapadr));
  }
  val ten_wrapnum() const {
    return val(typed_memory_view(_model->ptr()->ntendon * 1,
                                 _state->ptr()->ten_wrapnum));
  }
  val ten_J_rownnz() const {
    return val(typed_memory_view(_model->ptr()->ntendon * 1,
                                 _state->ptr()->ten_J_rownnz));
  }
  val ten_J_rowadr() const {
    return val(typed_memory_view(_model->ptr()->ntendon * 1,
                                 _state->ptr()->ten_J_rowadr));
  }
  val ten_J_colind() const {
    return val(typed_memory_view(_model->ptr()->ntendon * _model->ptr()->nv,
                                 _state->ptr()->ten_J_colind));
  }
  val ten_length() const {
    return val(typed_memory_view(_model->ptr()->ntendon * 1,
                                 _state->ptr()->ten_length));
  }
  val ten_J() const {
    return val(typed_memory_view(_model->ptr()->ntendon * _model->ptr()->nv,
                                 _state->ptr()->ten_J));
  }
  val wrap_obj() const {
    return val(
        typed_memory_view(_model->ptr()->nwrap * 2, _state->ptr()->wrap_obj));
  }
  val wrap_xpos() const {
    return val(
        typed_memory_view(_model->ptr()->nwrap * 6, _state->ptr()->wrap_xpos));
  }
  val actuator_length() const {
    return val(typed_memory_view(_model->ptr()->nu * 1,
                                 _state->ptr()->actuator_length));
  }
  val actuator_moment() const {
    return val(typed_memory_view(_model->ptr()->nu * _model->ptr()->nv,
                                 _state->ptr()->actuator_moment));
  }
  val crb() const {
    return val(
        typed_memory_view(_model->ptr()->nbody * 10, _state->ptr()->crb));
  }
  val qM() const {
    return val(typed_memory_view(_model->ptr()->nM * 1, _state->ptr()->qM));
  }
  val qLD() const {
    return val(typed_memory_view(_model->ptr()->nM * 1, _state->ptr()->qLD));
  }
  val qLDiagInv() const {
    return val(
        typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qLDiagInv));
  }
  val qLDiagSqrtInv() const {
    return val(
        typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qLDiagSqrtInv));
  }
  val ten_velocity() const {
    return val(typed_memory_view(_model->ptr()->ntendon * 1,
                                 _state->ptr()->ten_velocity));
  }
  val actuator_velocity() const {
    return val(typed_memory_view(_model->ptr()->nu * 1,
                                 _state->ptr()->actuator_velocity));
  }
  val cvel() const {
    return val(
        typed_memory_view(_model->ptr()->nbody * 6, _state->ptr()->cvel));
  }
  val cdof_dot() const {
    return val(
        typed_memory_view(_model->ptr()->nv * 6, _state->ptr()->cdof_dot));
  }
  val qfrc_bias() const {
    return val(
        typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qfrc_bias));
  }
  val qfrc_passive() const {
    return val(
        typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qfrc_passive));
  }
  val subtree_linvel() const {
    return val(typed_memory_view(_model->ptr()->nbody * 3,
                                 _state->ptr()->subtree_linvel));
  }
  val subtree_angmom() const {
    return val(typed_memory_view(_model->ptr()->nbody * 3,
                                 _state->ptr()->subtree_angmom));
  }
  val qH() const {
    return val(typed_memory_view(_model->ptr()->nM * 1, _state->ptr()->qH));
  }
  val qHDiagInv() const {
    return val(
        typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qHDiagInv));
  }
  val D_rownnz() const {
    return val(
        typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->D_rownnz));
  }
  val D_rowadr() const {
    return val(
        typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->D_rowadr));
  }
  val D_colind() const {
    return val(
        typed_memory_view(_model->ptr()->nD * 1, _state->ptr()->D_colind));
  }
  val qDeriv() const {
    return val(typed_memory_view(_model->ptr()->nD * 1, _state->ptr()->qDeriv));
  }
  val qLU() const {
    return val(typed_memory_view(_model->ptr()->nD * 1, _state->ptr()->qLU));
  }
  val actuator_force() const {
    return val(typed_memory_view(_model->ptr()->nu * 1,
                                 _state->ptr()->actuator_force));
  }
  val qfrc_actuator() const {
    return val(
        typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qfrc_actuator));
  }
  val qfrc_smooth() const {
    return val(
        typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qfrc_smooth));
  }
  val qacc_smooth() const {
    return val(
        typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qacc_smooth));
  }
  val qfrc_constraint() const {
    return val(typed_memory_view(_model->ptr()->nv * 1,
                                 _state->ptr()->qfrc_constraint));
  }
  val qfrc_inverse() const {
    return val(
        typed_memory_view(_model->ptr()->nv * 1, _state->ptr()->qfrc_inverse));
  }
  val cacc() const {
    return val(
        typed_memory_view(_model->ptr()->nbody * 6, _state->ptr()->cacc));
  }
  val cfrc_int() const {
    return val(
        typed_memory_view(_model->ptr()->nbody * 6, _state->ptr()->cfrc_int));
  }
  val cfrc_ext() const {
    return val(
        typed_memory_view(_model->ptr()->nbody * 6, _state->ptr()->cfrc_ext));
  }
  void freeLastXML() { return mj_freeLastXML(); }
  void step() { return mj_step(_model->ptr(), _state->ptr()); }
  void step1() { return mj_step1(_model->ptr(), _state->ptr()); }
  void step2() { return mj_step2(_model->ptr(), _state->ptr()); }
  void forward() { return mj_forward(_model->ptr(), _state->ptr()); }
  void inverse() { return mj_inverse(_model->ptr(), _state->ptr()); }
  void forwardSkip(int skipstage, int skipsensor) {
    return mj_forwardSkip(_model->ptr(), _state->ptr(), skipstage, skipsensor);
  }
  void inverseSkip(int skipstage, int skipsensor) {
    return mj_inverseSkip(_model->ptr(), _state->ptr(), skipstage, skipsensor);
  }
  void defaultSolRefImp(val solref, val solimp) {
    return mj_defaultSolRefImp(
        reinterpret_cast<mjtNum *>(solref["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(solimp["byteOffset"].as<int>()));
  }
  int sizeModel() { return mj_sizeModel(_model->ptr()); }
  void resetData() { return mj_resetData(_model->ptr(), _state->ptr()); }
  void resetDataDebug(unsigned char debug_value) {
    return mj_resetDataDebug(_model->ptr(), _state->ptr(), debug_value);
  }
  void resetDataKeyframe(int key) {
    return mj_resetDataKeyframe(_model->ptr(), _state->ptr(), key);
  }
  void deleteData() { return mj_deleteData(_state->ptr()); }
  void resetCallbacks() { return mj_resetCallbacks(); }
  void printFormattedModel(std::string filename, std::string float_format) {
    return mj_printFormattedModel(_model->ptr(), filename.c_str(),
                                  float_format.c_str());
  }
  void printModel(std::string filename) {
    return mj_printModel(_model->ptr(), filename.c_str());
  }
  void printFormattedData(std::string filename, std::string float_format) {
    return mj_printFormattedData(_model->ptr(), _state->ptr(), filename.c_str(),
                                 float_format.c_str());
  }
  void printData(std::string filename) {
    return mj_printData(_model->ptr(), _state->ptr(), filename.c_str());
  }
  void _printMat(val mat, int nr, int nc) {
    return mju_printMat(reinterpret_cast<mjtNum *>(mat["byteOffset"].as<int>()),
                        nr, nc);
  }
  void fwdPosition() { return mj_fwdPosition(_model->ptr(), _state->ptr()); }
  void fwdVelocity() { return mj_fwdVelocity(_model->ptr(), _state->ptr()); }
  void fwdActuation() { return mj_fwdActuation(_model->ptr(), _state->ptr()); }
  void fwdAcceleration() {
    return mj_fwdAcceleration(_model->ptr(), _state->ptr());
  }
  void fwdConstraint() {
    return mj_fwdConstraint(_model->ptr(), _state->ptr());
  }
  void Euler() { return mj_Euler(_model->ptr(), _state->ptr()); }
  void RungeKutta(int N) {
    return mj_RungeKutta(_model->ptr(), _state->ptr(), N);
  }
  void invPosition() { return mj_invPosition(_model->ptr(), _state->ptr()); }
  void invVelocity() { return mj_invVelocity(_model->ptr(), _state->ptr()); }
  void invConstraint() {
    return mj_invConstraint(_model->ptr(), _state->ptr());
  }
  void compareFwdInv() {
    return mj_compareFwdInv(_model->ptr(), _state->ptr());
  }
  void sensorPos() { return mj_sensorPos(_model->ptr(), _state->ptr()); }
  void sensorVel() { return mj_sensorVel(_model->ptr(), _state->ptr()); }
  void sensorAcc() { return mj_sensorAcc(_model->ptr(), _state->ptr()); }
  void energyPos() { return mj_energyPos(_model->ptr(), _state->ptr()); }
  void energyVel() { return mj_energyVel(_model->ptr(), _state->ptr()); }
  void checkPos() { return mj_checkPos(_model->ptr(), _state->ptr()); }
  void checkVel() { return mj_checkVel(_model->ptr(), _state->ptr()); }
  void checkAcc() { return mj_checkAcc(_model->ptr(), _state->ptr()); }
  void kinematics() { return mj_kinematics(_model->ptr(), _state->ptr()); }
  void comPos() { return mj_comPos(_model->ptr(), _state->ptr()); }
  void camlight() { return mj_camlight(_model->ptr(), _state->ptr()); }
  void tendon() { return mj_tendon(_model->ptr(), _state->ptr()); }
  void transmission() { return mj_transmission(_model->ptr(), _state->ptr()); }
  void crbCalculate() { return mj_crb(_model->ptr(), _state->ptr()); }
  void factorM() { return mj_factorM(_model->ptr(), _state->ptr()); }
  void solveM(val x, val y, int n) {
    return mj_solveM(_model->ptr(), _state->ptr(),
                     reinterpret_cast<mjtNum *>(x["byteOffset"].as<int>()),
                     reinterpret_cast<mjtNum *>(y["byteOffset"].as<int>()), n);
  }
  void solveM2(val x, val y, int n) {
    return mj_solveM2(_model->ptr(), _state->ptr(),
                      reinterpret_cast<mjtNum *>(x["byteOffset"].as<int>()),
                      reinterpret_cast<mjtNum *>(y["byteOffset"].as<int>()), n);
  }
  void comVel() { return mj_comVel(_model->ptr(), _state->ptr()); }
  void passive() { return mj_passive(_model->ptr(), _state->ptr()); }
  void subtreeVel() { return mj_subtreeVel(_model->ptr(), _state->ptr()); }
  void rne(int flg_acc, val result) {
    return mj_rne(_model->ptr(), _state->ptr(), flg_acc,
                  reinterpret_cast<mjtNum *>(result["byteOffset"].as<int>()));
  }
  void rnePostConstraint() {
    return mj_rnePostConstraint(_model->ptr(), _state->ptr());
  }
  void collision() { return mj_collision(_model->ptr(), _state->ptr()); }
  void makeConstraint() {
    return mj_makeConstraint(_model->ptr(), _state->ptr());
  }
  void projectConstraint() {
    return mj_projectConstraint(_model->ptr(), _state->ptr());
  }
  void referenceConstraint() {
    return mj_referenceConstraint(_model->ptr(), _state->ptr());
  }
  int isPyramidal() { return mj_isPyramidal(_model->ptr()); }
  int isSparse() { return mj_isSparse(_model->ptr()); }
  int isDual() { return mj_isDual(_model->ptr()); }
  void mulJacVec(val res, val vec) {
    return mj_mulJacVec(
        _model->ptr(), _state->ptr(),
        reinterpret_cast<mjtNum *>(res["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(vec["byteOffset"].as<int>()));
  }
  void mulJacTVec(val res, val vec) {
    return mj_mulJacTVec(
        _model->ptr(), _state->ptr(),
        reinterpret_cast<mjtNum *>(res["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(vec["byteOffset"].as<int>()));
  }
  void jacSubtreeCom(val jacp, int body) {
    return mj_jacSubtreeCom(
        _model->ptr(), _state->ptr(),
        reinterpret_cast<mjtNum *>(jacp["byteOffset"].as<int>()), body);
  }
  int name2id(int type, std::string name) {
    return mj_name2id(_model->ptr(), type, name.c_str());
  }
  std::string id2name(int type, int id) {
    return std::string(mj_id2name(_model->ptr(), type, id));
  }
  void fullM(val dst, val M) {
    return mj_fullM(_model->ptr(),
                    reinterpret_cast<mjtNum *>(dst["byteOffset"].as<int>()),
                    reinterpret_cast<mjtNum *>(M["byteOffset"].as<int>()));
  }
  void differentiatePos(val qvel, mjtNum dt, val qpos1, val qpos2) {
    return mj_differentiatePos(
        _model->ptr(), reinterpret_cast<mjtNum *>(qvel["byteOffset"].as<int>()),
        dt, reinterpret_cast<mjtNum *>(qpos1["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(qpos2["byteOffset"].as<int>()));
  }
  void integratePos(val qpos, val qvel, mjtNum dt) {
    return mj_integratePos(
        _model->ptr(), reinterpret_cast<mjtNum *>(qpos["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(qvel["byteOffset"].as<int>()), dt);
  }
  void normalizeQuat(val qpos) {
    return mj_normalizeQuat(_model->ptr(), reinterpret_cast<mjtNum *>(
                                               qpos["byteOffset"].as<int>()));
  }
  mjtNum getTotalmass() { return mj_getTotalmass(_model->ptr()); }
  std::string getPluginConfig(int plugin_id, std::string attrib) {
    return std::string(
        mj_getPluginConfig(_model->ptr(), plugin_id, attrib.c_str()));
  }
  void loadPluginLibrary(std::string path) {
    return mj_loadPluginLibrary(path.c_str());
  }
  int version() { return mj_version(); }
  std::string versionString() { return std::string(mj_versionString()); }
  void _rectangle(mjrRect viewport, float r, float g, float b, float a) {
    return mjr_rectangle(viewport, r, g, b, a);
  }
  void _finish() { return mjr_finish(); }
  int _getError() { return mjr_getError(); }
  mjuiThemeSpacing i_themeSpacing(int ind) { return mjui_themeSpacing(ind); }
  mjuiThemeColor i_themeColor(int ind) { return mjui_themeColor(ind); }
  void _error(std::string msg) { return mju_error(msg.c_str()); }
  void _error_i(std::string msg, int i) { return mju_error_i(msg.c_str(), i); }
  void _error_s(std::string msg, std::string text) {
    return mju_error_s(msg.c_str(), text.c_str());
  }
  void _warning(std::string msg) { return mju_warning(msg.c_str()); }
  void _warning_i(std::string msg, int i) {
    return mju_warning_i(msg.c_str(), i);
  }
  void _warning_s(std::string msg, std::string text) {
    return mju_warning_s(msg.c_str(), text.c_str());
  }
  void _clearHandlers() { return mju_clearHandlers(); }
  void warning(int warning, int info) {
    return mj_warning(_state->ptr(), warning, info);
  }
  void _writeLog(std::string type, std::string msg) {
    return mju_writeLog(type.c_str(), msg.c_str());
  }
  int activate(std::string filename) { return mj_activate(filename.c_str()); }
  void deactivate() { return mj_deactivate(); }
  void _zero(val res, int n) {
    return mju_zero(reinterpret_cast<mjtNum *>(res["byteOffset"].as<int>()), n);
  }
  void _fill(val res, mjtNum val, int n) {
    return mju_fill(reinterpret_cast<mjtNum *>(res["byteOffset"].as<int>()),
                    val, n);
  }
  void _copy(val res, val data, int n) {
    return mju_copy(reinterpret_cast<mjtNum *>(res["byteOffset"].as<int>()),
                    reinterpret_cast<mjtNum *>(data["byteOffset"].as<int>()),
                    n);
  }
  mjtNum _sum(val vec, int n) {
    return mju_sum(reinterpret_cast<mjtNum *>(vec["byteOffset"].as<int>()), n);
  }
  mjtNum _L1(val vec, int n) {
    return mju_L1(reinterpret_cast<mjtNum *>(vec["byteOffset"].as<int>()), n);
  }
  void _scl(val res, val vec, mjtNum scl, int n) {
    return mju_scl(reinterpret_cast<mjtNum *>(res["byteOffset"].as<int>()),
                   reinterpret_cast<mjtNum *>(vec["byteOffset"].as<int>()), scl,
                   n);
  }
  void _add(val res, val vec1, val vec2, int n) {
    return mju_add(reinterpret_cast<mjtNum *>(res["byteOffset"].as<int>()),
                   reinterpret_cast<mjtNum *>(vec1["byteOffset"].as<int>()),
                   reinterpret_cast<mjtNum *>(vec2["byteOffset"].as<int>()), n);
  }
  void _sub(val res, val vec1, val vec2, int n) {
    return mju_sub(reinterpret_cast<mjtNum *>(res["byteOffset"].as<int>()),
                   reinterpret_cast<mjtNum *>(vec1["byteOffset"].as<int>()),
                   reinterpret_cast<mjtNum *>(vec2["byteOffset"].as<int>()), n);
  }
  void _addTo(val res, val vec, int n) {
    return mju_addTo(reinterpret_cast<mjtNum *>(res["byteOffset"].as<int>()),
                     reinterpret_cast<mjtNum *>(vec["byteOffset"].as<int>()),
                     n);
  }
  void _subFrom(val res, val vec, int n) {
    return mju_subFrom(reinterpret_cast<mjtNum *>(res["byteOffset"].as<int>()),
                       reinterpret_cast<mjtNum *>(vec["byteOffset"].as<int>()),
                       n);
  }
  void _addToScl(val res, val vec, mjtNum scl, int n) {
    return mju_addToScl(reinterpret_cast<mjtNum *>(res["byteOffset"].as<int>()),
                        reinterpret_cast<mjtNum *>(vec["byteOffset"].as<int>()),
                        scl, n);
  }
  void _addScl(val res, val vec1, val vec2, mjtNum scl, int n) {
    return mju_addScl(reinterpret_cast<mjtNum *>(res["byteOffset"].as<int>()),
                      reinterpret_cast<mjtNum *>(vec1["byteOffset"].as<int>()),
                      reinterpret_cast<mjtNum *>(vec2["byteOffset"].as<int>()),
                      scl, n);
  }
  mjtNum _normalize(val res, int n) {
    return mju_normalize(
        reinterpret_cast<mjtNum *>(res["byteOffset"].as<int>()), n);
  }
  mjtNum _norm(val res, int n) {
    return mju_norm(reinterpret_cast<mjtNum *>(res["byteOffset"].as<int>()), n);
  }
  mjtNum _dot(val vec1, val vec2, int n) {
    return mju_dot(reinterpret_cast<mjtNum *>(vec1["byteOffset"].as<int>()),
                   reinterpret_cast<mjtNum *>(vec2["byteOffset"].as<int>()), n);
  }
  void _mulMatVec(val res, val mat, val vec, int nr, int nc) {
    return mju_mulMatVec(
        reinterpret_cast<mjtNum *>(res["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(mat["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(vec["byteOffset"].as<int>()), nr, nc);
  }
  void _mulMatTVec(val res, val mat, val vec, int nr, int nc) {
    return mju_mulMatTVec(
        reinterpret_cast<mjtNum *>(res["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(mat["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(vec["byteOffset"].as<int>()), nr, nc);
  }
  mjtNum _mulVecMatVec(val vec1, val mat, val vec2, int n) {
    return mju_mulVecMatVec(
        reinterpret_cast<mjtNum *>(vec1["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(mat["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(vec2["byteOffset"].as<int>()), n);
  }
  void _transpose(val res, val mat, int nr, int nc) {
    return mju_transpose(
        reinterpret_cast<mjtNum *>(res["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(mat["byteOffset"].as<int>()), nr, nc);
  }
  void _symmetrize(val res, val mat, int n) {
    return mju_symmetrize(
        reinterpret_cast<mjtNum *>(res["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(mat["byteOffset"].as<int>()), n);
  }
  void _eye(val mat, int n) {
    return mju_eye(reinterpret_cast<mjtNum *>(mat["byteOffset"].as<int>()), n);
  }
  void _mulMatMat(val res, val mat1, val mat2, int r1, int c1, int c2) {
    return mju_mulMatMat(
        reinterpret_cast<mjtNum *>(res["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(mat1["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(mat2["byteOffset"].as<int>()), r1, c1, c2);
  }
  void _mulMatMatT(val res, val mat1, val mat2, int r1, int c1, int r2) {
    return mju_mulMatMatT(
        reinterpret_cast<mjtNum *>(res["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(mat1["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(mat2["byteOffset"].as<int>()), r1, c1, r2);
  }
  void _mulMatTMat(val res, val mat1, val mat2, int r1, int c1, int c2) {
    return mju_mulMatTMat(
        reinterpret_cast<mjtNum *>(res["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(mat1["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(mat2["byteOffset"].as<int>()), r1, c1, c2);
  }
  void _sqrMatTD(val res, val mat, val diag, int nr, int nc) {
    return mju_sqrMatTD(
        reinterpret_cast<mjtNum *>(res["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(mat["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(diag["byteOffset"].as<int>()), nr, nc);
  }
  int _cholFactor(val mat, int n, mjtNum mindiag) {
    return mju_cholFactor(
        reinterpret_cast<mjtNum *>(mat["byteOffset"].as<int>()), n, mindiag);
  }
  void _cholSolve(val res, val mat, val vec, int n) {
    return mju_cholSolve(
        reinterpret_cast<mjtNum *>(res["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(mat["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(vec["byteOffset"].as<int>()), n);
  }
  int _cholUpdate(val mat, val x, int n, int flg_plus) {
    return mju_cholUpdate(
        reinterpret_cast<mjtNum *>(mat["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(x["byteOffset"].as<int>()), n, flg_plus);
  }
  void _encodePyramid(val pyramid, val force, val mu, int dim) {
    return mju_encodePyramid(
        reinterpret_cast<mjtNum *>(pyramid["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(force["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(mu["byteOffset"].as<int>()), dim);
  }
  void _decodePyramid(val force, val pyramid, val mu, int dim) {
    return mju_decodePyramid(
        reinterpret_cast<mjtNum *>(force["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(pyramid["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(mu["byteOffset"].as<int>()), dim);
  }
  mjtNum _springDamper(mjtNum pos0, mjtNum vel0, mjtNum Kp, mjtNum Kv,
                       mjtNum dt) {
    return mju_springDamper(pos0, vel0, Kp, Kv, dt);
  }
  mjtNum _min(mjtNum a, mjtNum b) { return mju_min(a, b); }
  mjtNum _max(mjtNum a, mjtNum b) { return mju_max(a, b); }
  mjtNum _clip(mjtNum x, mjtNum min, mjtNum max) {
    return mju_clip(x, min, max);
  }
  mjtNum _sign(mjtNum x) { return mju_sign(x); }
  int _round(mjtNum x) { return mju_round(x); }
  std::string _type2Str(int type) { return std::string(mju_type2Str(type)); }
  int _str2Type(std::string str) { return mju_str2Type(str.c_str()); }
  std::string _writeNumBytes(size_t nbytes) {
    return std::string(mju_writeNumBytes(nbytes));
  }
  std::string _warningText(int warning, size_t info) {
    return std::string(mju_warningText(warning, info));
  }
  int _isBad(mjtNum x) { return mju_isBad(x); }
  int _isZero(val vec, int n) {
    return mju_isZero(reinterpret_cast<mjtNum *>(vec["byteOffset"].as<int>()),
                      n);
  }
  mjtNum _standardNormal(val num2) {
    return mju_standardNormal(
        reinterpret_cast<mjtNum *>(num2["byteOffset"].as<int>()));
  }
  void _insertionSort(val list, int n) {
    return mju_insertionSort(
        reinterpret_cast<mjtNum *>(list["byteOffset"].as<int>()), n);
  }
  mjtNum _Halton(int index, int base) { return mju_Halton(index, base); }
  mjtNum _sigmoid(mjtNum x) { return mju_sigmoid(x); }
  void _transitionFD(mjtNum eps, mjtByte centered, val A, val B, val C, val D) {
    return mjd_transitionFD(
        _model->ptr(), _state->ptr(), eps, centered,
        reinterpret_cast<mjtNum *>(A["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(B["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(C["byteOffset"].as<int>()),
        reinterpret_cast<mjtNum *>(D["byteOffset"].as<int>()));
  }
  int _pluginCount() { return mjp_pluginCount(); }

private:
  Model *_model;
  State *_state;
};

// main function
int main(int argc, char **argv) {
  std::printf("MuJoCo version: %d\n\n", mj_version());
  return 0;
}
