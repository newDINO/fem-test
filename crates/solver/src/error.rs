use thiserror::Error;

#[derive(Debug, Error)]
pub enum FemError {
    #[error("mesh is empty")]
    EmptyMesh,
    #[error("invalid material: {0}")]
    InvalidMaterial(String),
    #[error("triangle {0} is degenerate or has repeated nodes")]
    DegenerateElement(usize),
    #[error("not enough constraints: only {count} scalar constraint(s)")]
    InsufficientConstraints { count: usize },
    #[error("stiffness matrix is singular: {0}")]
    SingularMatrix(String),
    #[error("STL import failed: {0}")]
    Stl(String),
    #[error("mesh is not a single axis-aligned plane: {0}")]
    NonPlanar(String),
    #[error("invalid input: {0}")]
    InvalidInput(String),
}
