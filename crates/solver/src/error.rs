use fem_types::MeshError;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum FemError {
    #[error(transparent)]
    Mesh(#[from] MeshError),
    #[error("invalid material: {0}")]
    InvalidMaterial(String),
    #[error("not enough constraints: only {count} scalar constraint(s)")]
    InsufficientConstraints { count: usize },
    #[error("inconsistent linear constraint: {0}")]
    ConstraintConflict(String),
    #[error("stiffness matrix is singular: {0}")]
    SingularMatrix(String),
    #[error("invalid input: {0}")]
    InvalidInput(String),
}
